/*
 * 虚拟档位（detent）旋钮手感算法
 *
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#include "knob_haptic.h"

#include <math.h>
#include <string.h>

#include <zephyr/sys/util.h>

/* 把角度折算到 [-period/2, period/2)，用于求相对最近档位中心的偏移 */
static float wrap_to_period(float angle, float period)
{
	float wrapped = fmodf(angle, period);

	if (wrapped > period * 0.5f) {
		wrapped -= period;
	} else if (wrapped < -period * 0.5f) {
		wrapped += period;
	}

	return wrapped;
}

static int32_t detent_of(float angle_deg, float spacing_deg)
{
	if (spacing_deg <= 0.0f) {
		return 0;
	}

	return (int32_t)floorf(angle_deg / spacing_deg + 0.5f);
}

void knob_haptic_init(struct knob_haptic *knob, const struct knob_feel_params *params,
		      float angle_deg, uint32_t now_ms)
{
	if (knob == NULL) {
		return;
	}

	memset(knob, 0, sizeof(*knob));
	if (params != NULL) {
		knob->params = *params;
	}

	knob->angle_deg = angle_deg;
	knob->last_time_ms = now_ms;
	knob->detent_index = detent_of(angle_deg, knob->params.detent_spacing_deg);
	knob->primed = true;
}

float knob_haptic_update(struct knob_haptic *knob, float angle_deg, float rpm, uint32_t now_ms)
{
	if (knob == NULL) {
		return 0.0f;
	}

	const struct knob_feel_params *p = &knob->params;

	/* 参数非法时保持输出为 0，避免除零与无界输出 */
	if (p->detent_spacing_deg <= 0.0f || p->max_torque <= 0.0f) {
		knob->torque = 0.0f;
		return 0.0f;
	}

	float dt = (float)(int32_t)(now_ms - knob->last_time_ms) * 0.001f;

	/* 同一毫秒内重复调用时按 1 ms 处理，保证算法结果可复现 */
	if (dt <= 0.0f) {
		dt = 0.001f;
	} else if (dt > 0.1f) {
		/* 线程被长时间阻塞后不要用超大 dt 积分，避免力矩突变 */
		dt = 0.1f;
	}
	knob->last_time_ms = now_ms;

	/* 转速低通：C620 的转速反馈量化到 1 rpm，直接做阻尼会引入抖动 */
	float alpha = CLAMP(p->velocity_lpf_alpha, 0.0f, 0.999f);

	knob->rpm_filt = alpha * knob->rpm_filt + (1.0f - alpha) * rpm;

	/* 1. 虚拟弹簧：把旋钮拉回最近的档位中心，天然具有周期性 */
	float offset = wrap_to_period(angle_deg, p->detent_spacing_deg);

	if (fabsf(offset) < p->deadband_deg) {
		offset = 0.0f;
	}

	float torque = -p->spring_stiffness * offset;

	/* 2. 阻尼力矩：始终与转速反向，抑制快速旋转时的振荡 */
	torque -= p->damping * knob->rpm_filt;

	/* 3. 越档检测：档位编号变化且越过回滞带才算一次越档，
	 *    快速旋转时允许一个周期内跨过多个档位。
	 */
	int32_t raw_index = detent_of(angle_deg, p->detent_spacing_deg);
	int32_t delta = raw_index - knob->detent_index;

	if (delta != 0) {
		float edge = (delta > 0 ? (float)knob->detent_index + 0.5f
					: (float)knob->detent_index - 0.5f) *
			     p->detent_spacing_deg;
		bool crossed = (delta > 0) ? (angle_deg >= edge + p->crossing_hysteresis_deg)
					   : (angle_deg <= edge - p->crossing_hysteresis_deg);

		if (crossed) {
			knob->detent_index = raw_index;
			knob->crossings += (uint32_t)(delta > 0 ? delta : -delta);

			/* 4. 越档顿挫脉冲：方向与转动方向相反，形成“咔哒”阻尼感。
			 *    衰减过程在下方统一处理。
			 */
			if (p->detent_impulse > 0.0f) {
				knob->impulse = (delta > 0) ? -p->detent_impulse : p->detent_impulse;
			}
		}
	}

	/* 越档脉冲按指数衰减 */
	if (knob->impulse != 0.0f) {
		if (p->impulse_decay_ms > 0.0f) {
			knob->impulse *= expf(-dt * 1000.0f / p->impulse_decay_ms);
		} else {
			knob->impulse = 0.0f;
		}
	}

	torque += knob->impulse;

	/* 5. 力矩限幅 */
	torque = CLAMP(torque, -p->max_torque, p->max_torque);

	/* 6. 力矩变化率限幅：抑制电流指令突变，降低机械冲击与振荡风险 */
	if (p->torque_slew_rate > 0.0f) {
		float max_step = p->torque_slew_rate * dt;

		torque = CLAMP(torque, knob->torque - max_step, knob->torque + max_step);
	}

	if (!isfinite(torque)) {
		torque = 0.0f;
	}

	knob->angle_deg = angle_deg;
	knob->torque = torque;

	return torque;
}
