/*
 * 虚拟档位（detent）旋钮手感算法
 *
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef KNOB_HAPTIC_H_
#define KNOB_HAPTIC_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 档位手感参数
 *
 * 角度与转速均为输出轴侧数值（与 motor_status_t 一致），力矩单位为 N·m，
 * 同样是输出轴侧力矩。所有参数在运行时只读，修改后需重新初始化。
 */
struct knob_feel_params {
	/** 相邻档位中心夹角，单位 deg，必须 > 0 */
	float detent_spacing_deg;
	/** 虚拟弹簧刚度，单位 N·m/deg；档位边界处力矩约为 stiffness * spacing / 2 */
	float spring_stiffness;
	/** 阻尼系数，单位 N·m/rpm，输出力矩与转速反向 */
	float damping;
	/** 越档脉冲峰值力矩，单位 N·m；为 0 时关闭越档顿挫 */
	float detent_impulse;
	/** 越档脉冲指数衰减时间常数，单位 ms，必须 > 0 */
	float impulse_decay_ms;
	/** 输出力矩限幅，单位 N·m，必须 > 0 */
	float max_torque;
	/** 力矩变化率限幅，单位 N·m/s；为 0 时不做限制 */
	float torque_slew_rate;
	/** 转速低通系数 [0, 1)，越大越平滑，用于抑制 CAN 反馈量化噪声 */
	float velocity_lpf_alpha;
	/** 越档回滞，单位 deg，抑制档位边界处的来回抖动 */
	float crossing_hysteresis_deg;
	/** 档位中心死区，单位 deg，避免静摩擦下在中心点附近微振 */
	float deadband_deg;
};

/**
 * @brief 虚拟档位手感运行状态
 *
 * 由 knob_haptic_init() 初始化，之后每个控制周期调用 knob_haptic_update()。
 * 字段对外只读，便于上层做遥测与调试。
 */
struct knob_haptic {
	struct knob_feel_params params;
	/** 上一周期的角度 */
	float angle_deg;
	/** 低通后的转速 */
	float rpm_filt;
	/** 当前越档脉冲力矩（带符号） */
	float impulse;
	/** 上一周期输出的力矩 */
	float torque;
	/** 当前档位编号，档位中心为 index * detent_spacing_deg */
	int32_t detent_index;
	uint32_t last_time_ms;
	/** 累计越档次数 */
	uint32_t crossings;
	bool primed;
};

/**
 * @brief 初始化手感状态
 *
 * 以传入角度作为当前档位，清零越档计数。
 *
 * @param knob      手感状态
 * @param params    手感参数
 * @param angle_deg 当前输出轴角度，单位 deg
 * @param now_ms    当前时间，单位 ms
 */
void knob_haptic_init(struct knob_haptic *knob, const struct knob_feel_params *params,
		      float angle_deg, uint32_t now_ms);

/**
 * @brief 计算一个控制周期的输出力矩
 *
 * 输出 = 虚拟弹簧力矩 + 阻尼力矩 + 越档脉冲力矩，并依次经过
 * 力矩限幅与力矩变化率限幅。
 *
 * @param knob      手感状态
 * @param angle_deg 当前输出轴角度，单位 deg（应使用多圈累计角度）
 * @param rpm       当前输出轴转速，单位 rpm
 * @param now_ms    当前时间，单位 ms
 * @return 输出轴力矩指令，单位 N·m
 */
float knob_haptic_update(struct knob_haptic *knob, float angle_deg, float rpm, uint32_t now_ms);

/** @brief 当前档位编号 */
static inline int32_t knob_haptic_detent(const struct knob_haptic *knob)
{
	return knob->detent_index;
}

/** @brief 累计越档次数 */
static inline uint32_t knob_haptic_crossings(const struct knob_haptic *knob)
{
	return knob->crossings;
}

#endif /* KNOB_HAPTIC_H_ */
