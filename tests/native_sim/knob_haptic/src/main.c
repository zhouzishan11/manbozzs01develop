/*
 * 虚拟档位手感算法单元测试
 *
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <zephyr/ztest.h>

#include "knob_haptic.h"

#define TOL 1.0e-4f

static struct knob_feel_params make_params(void)
{
	return (struct knob_feel_params){
		.detent_spacing_deg = 30.0f,
		.spring_stiffness = 0.02f,
		.damping = 0.001f,
		.detent_impulse = 0.4f,
		.impulse_decay_ms = 10.0f,
		.max_torque = 1.0f,
		.torque_slew_rate = 0.0f,
		.velocity_lpf_alpha = 0.0f,
		.crossing_hysteresis_deg = 1.0f,
		.deadband_deg = 0.0f,
	};
}

ZTEST(knob_haptic, test_spring_pulls_back_to_center)
{
	struct knob_feel_params p = make_params();
	struct knob_haptic k;

	knob_haptic_init(&k, &p, 5.0f, 0);
	zassert_within(knob_haptic_update(&k, 5.0f, 0.0f, 1), -0.1f, TOL,
		       "弹簧应把旋钮拉回档位中心");

	/* 偏差换到另一侧，力矩方向应相反 */
	zassert_within(knob_haptic_update(&k, -5.0f, 0.0f, 2), 0.1f, TOL, "力矩方向应与偏差相反");
}

ZTEST(knob_haptic, test_damping_opposes_speed)
{
	struct knob_feel_params p = make_params();
	struct knob_haptic k;

	knob_haptic_init(&k, &p, 0.0f, 0);
	zassert_within(knob_haptic_update(&k, 0.0f, 100.0f, 1), -0.1f, TOL, "正转时应输出反向阻尼");

	knob_haptic_init(&k, &p, 0.0f, 0);
	zassert_within(knob_haptic_update(&k, 0.0f, -100.0f, 1), 0.1f, TOL, "反转时应输出反向阻尼");
}

ZTEST(knob_haptic, test_angle_wrap_at_negative_angle)
{
	struct knob_feel_params p = make_params();
	struct knob_haptic k;

	/* -29° 距离 -30° 档位中心 +1°，弹簧应输出负力矩 */
	knob_haptic_init(&k, &p, -29.0f, 0);
	zassert_within(knob_haptic_update(&k, -29.0f, 0.0f, 1), -0.02f, TOL, "负角度回绕错误");
	zassert_equal(knob_haptic_detent(&k), -1);
}

ZTEST(knob_haptic, test_angle_wrap_at_multi_turn_angle)
{
	struct knob_feel_params p = make_params();
	struct knob_haptic k;

	knob_haptic_init(&k, &p, 3599.0f, 0);
	zassert_equal(knob_haptic_detent(&k), 120);

	/* 3616.5° 与 16.5° 等价，应识别为一次越档 */
	knob_haptic_update(&k, 3616.5f, 0.0f, 1);
	zassert_equal(knob_haptic_detent(&k), 121);
	zassert_equal(knob_haptic_crossings(&k), 1);
}

ZTEST(knob_haptic, test_detent_crossing_fires_pulse)
{
	struct knob_feel_params p = make_params();
	struct knob_haptic k;

	knob_haptic_init(&k, &p, 14.0f, 0);
	knob_haptic_update(&k, 14.0f, 0.0f, 1);
	zassert_equal(knob_haptic_crossings(&k), 0);

	/* 越过 15° 边界并超过 1° 回滞，触发越档脉冲 */
	float torque = knob_haptic_update(&k, 16.5f, 0.0f, 2);

	zassert_equal(knob_haptic_detent(&k), 1);
	zassert_equal(knob_haptic_crossings(&k), 1);
	zassert_true(k.impulse < 0.0f, "正向越档时脉冲应反向");
	zassert_true(torque < 0.0f, "越档瞬间脉冲应压过弹簧力矩");

	/* 脉冲衰减后只剩弹簧力矩：offset = 16.5 - 30 = -13.5° */
	for (int i = 0; i < 100; i++) {
		torque = knob_haptic_update(&k, 16.5f, 0.0f, (uint32_t)(3 + i));
	}
	zassert_within(torque, 0.27f, TOL, "脉冲未按预期衰减");
}

ZTEST(knob_haptic, test_crossing_hysteresis_blocks_chatter)
{
	struct knob_feel_params p = make_params();
	struct knob_haptic k;

	knob_haptic_init(&k, &p, 14.0f, 0);
	knob_haptic_update(&k, 14.0f, 0.0f, 1);

	/* 越过边界但未超过回滞带，不应计为越档 */
	knob_haptic_update(&k, 15.5f, 0.0f, 2);
	knob_haptic_update(&k, 15.9f, 0.0f, 3);
	zassert_equal(knob_haptic_detent(&k), 0);
	zassert_equal(knob_haptic_crossings(&k), 0);

	knob_haptic_update(&k, 16.1f, 0.0f, 4);
	zassert_equal(knob_haptic_detent(&k), 1);
	zassert_equal(knob_haptic_crossings(&k), 1);

	/* 在 30° 档位中心附近小幅抖动不应产生新的越档 */
	for (uint32_t i = 0; i < 10; i++) {
		knob_haptic_update(&k, 29.9f, 0.0f, 5 + i * 2);
		knob_haptic_update(&k, 30.4f, 0.0f, 6 + i * 2);
	}
	zassert_equal(knob_haptic_crossings(&k), 1);
}

ZTEST(knob_haptic, test_fast_rotation_counts_all_detents)
{
	struct knob_feel_params p = make_params();
	struct knob_haptic k;

	knob_haptic_init(&k, &p, 0.0f, 0);

	/* 一个控制周期内转过 100°，跨过 3 个档位 */
	knob_haptic_update(&k, 100.0f, 0.0f, 1);
	zassert_equal(knob_haptic_detent(&k), 3);
	zassert_equal(knob_haptic_crossings(&k), 3);
}

ZTEST(knob_haptic, test_reverse_crossing_pulse_direction)
{
	struct knob_feel_params p = make_params();
	struct knob_haptic k;

	knob_haptic_init(&k, &p, 16.0f, 0);
	knob_haptic_update(&k, 13.5f, 0.0f, 1);

	zassert_equal(knob_haptic_detent(&k), 0);
	zassert_equal(knob_haptic_crossings(&k), 1);
	zassert_true(k.impulse > 0.0f, "反向越档时脉冲应正向");
}

ZTEST(knob_haptic, test_torque_clamped_to_max)
{
	struct knob_feel_params p = make_params();

	p.spring_stiffness = 1.0f;

	struct knob_haptic k;

	knob_haptic_init(&k, &p, 0.0f, 0);
	zassert_within(knob_haptic_update(&k, 14.9f, 0.0f, 1), -1.0f, TOL, "正向力矩未限幅");
	zassert_within(knob_haptic_update(&k, -14.9f, 0.0f, 2), 1.0f, TOL, "反向力矩未限幅");
}

ZTEST(knob_haptic, test_torque_slew_rate_limit)
{
	struct knob_feel_params p = make_params();

	p.spring_stiffness = 0.02f;
	p.torque_slew_rate = 10.0f; /* 每 1 ms 最多变化 0.01 N·m */

	struct knob_haptic k;

	knob_haptic_init(&k, &p, 14.9f, 0);
	float t1 = knob_haptic_update(&k, 14.9f, 0.0f, 1);
	float t2 = knob_haptic_update(&k, 14.9f, 0.0f, 2);

	zassert_true(fabsf(t1 + 0.01f) < TOL, "首个周期应受变化率限制: t1=%.6f", (double)t1);
	zassert_true(fabsf(t2 + 0.02f) < TOL, "变化率限制未累加: t2=%.6f", (double)t2);
}

ZTEST(knob_haptic, test_deadband_suppresses_output_at_center)
{
	struct knob_feel_params p = make_params();

	p.deadband_deg = 0.5f;

	struct knob_haptic k;

	knob_haptic_init(&k, &p, 0.0f, 0);
	zassert_within(knob_haptic_update(&k, 0.3f, 0.0f, 1), 0.0f, TOL, "死区内不应输出弹簧力矩");
	zassert_true(knob_haptic_update(&k, 0.6f, 0.0f, 2) < 0.0f, "死区外应恢复弹簧力矩");
}

ZTEST(knob_haptic, test_invalid_params_output_zero)
{
	struct knob_feel_params p = {0};
	struct knob_haptic k;

	knob_haptic_init(&k, &p, 0.0f, 0);
	zassert_within(knob_haptic_update(&k, 10.0f, 100.0f, 1), 0.0f, TOL, "参数非法时应输出 0");
}

ZTEST_SUITE(knob_haptic, NULL, NULL, NULL, NULL, NULL);
