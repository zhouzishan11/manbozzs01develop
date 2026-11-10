/*
 * DJI M3508 飞机旋钮虚拟档位手感示例
 *
 * 通过虚拟弹簧 + 阻尼 + 越档顿挫脉冲，在没有机械档位的旋钮上生成
 * “一档一档”的力矩反馈。手感参数集中在 knob_params 中，可整体替换以扩展
 * 其他旋钮手感。
 *
 * 力矩下发路径：
 *   手感算法输出力矩(N·m) -> motor_set(VO / MOTOR_TARGET_TORQUE)
 *   -> 驱动按 gear_ratio 与 TORQUE2CURRENT 换算为电流指令
 *   -> C620 电调内部电流环执行
 *
 * M3508 没有力矩传感器，motor_status_t.torque 是由反馈电流换算出的估计值，
 * 因此这里无法做真正的力矩闭环，只能做“力矩指令 -> 电流闭环”。
 *
 * Copyright (c) 2024 ttwards
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "knob_haptic.h"

LOG_MODULE_REGISTER(dji_m3508_demo, LOG_LEVEL_INF);

// 电机设备定义
#define M3508_NODE DT_NODELABEL(m3508_motor)
static const struct device *const m3508_motor = DEVICE_DT_GET(M3508_NODE);

// 控制周期：1 kHz，与 C620 的 1 kHz 周期反馈匹配
#define KNOB_CONTROL_PERIOD_MS 1
// 安全转速：超过后切断输出力矩，避免手感算法在异常高速下继续加力
#define KNOB_SAFE_MAX_RPM      300.0f

/*
 * 手感参数（输出轴侧，30°/档，一圈 12 档）
 *
 * 档位边界处弹簧力矩 = spring_stiffness * detent_spacing_deg / 2 = 0.375 N·m，
 * 峰值仍受 max_torque 限制，便于在低减速比/高负载机构上快速调参。
 */
static const struct knob_feel_params knob_params = {
	.detent_spacing_deg = 30.0f,     // 档位间距
	.spring_stiffness = 0.025f,      // 虚拟弹簧刚度 N·m/deg
	.damping = 0.0015f,              // 阻尼 N·m/rpm，100 rpm 时约 0.15 N·m
	.detent_impulse = 0.45f,         // 越档顿挫脉冲峰值 N·m
	.impulse_decay_ms = 12.0f,       // 脉冲衰减时间常数 ms
	.max_torque = 1.2f,              // 力矩限幅 N·m
	.torque_slew_rate = 120.0f,      // 力矩变化率限幅 N·m/s
	.velocity_lpf_alpha = 0.85f,     // 转速低通系数
	.crossing_hysteresis_deg = 1.0f, // 越档回滞
	.deadband_deg = 0.2f,            // 档位中心死区
};

static struct knob_haptic knob;
static volatile bool knob_ready;

/**
 * @brief 下发输出轴力矩指令
 *
 * 同时写入力矩限幅，覆盖驱动默认的 ±99999，防止异常参数下电流打满。
 */
static void knob_apply_torque(float torque_nm)
{
	motor_setpoint_t setpoint = {
		.torque = torque_nm,
		.mode = VO,
		.target = MOTOR_TARGET_TORQUE,
		.torque_limit = {-knob_params.max_torque, knob_params.max_torque},
		.speed_limit = {-KNOB_SAFE_MAX_RPM, KNOB_SAFE_MAX_RPM},
	};

	(void)motor_set(m3508_motor, &setpoint);
}

/**
 * @brief 1 kHz 手感控制线程
 */
static void knob_control_thread(void *arg1, void *arg2, void *arg3)
{
	static bool link_was_online;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		k_msleep(KNOB_CONTROL_PERIOD_MS);

		if (!knob_ready) {
			continue;
		}

		motor_status_t status;

		if (motor_get(m3508_motor, &status) != 0) {
			knob_apply_torque(0.0f);
			link_was_online = false;
			continue;
		}

		/* 离线时驱动不会下发电流；超速时主动切断，避免失控 */
		if (!status.online) {
			knob_apply_torque(0.0f);
			link_was_online = false;
			continue;
		}

		if (!link_was_online) {
			/* 重新上线：以当前位置重新对齐档位，避免断线期间转动
			 * 造成的角度跳变与力矩冲击。
			 */
			knob_haptic_init(&knob, &knob_params, status.sum_angle,
					 k_uptime_get_32());
			link_was_online = true;
		}

		if (!status.enabled || fabsf(status.rpm) > KNOB_SAFE_MAX_RPM) {
			knob_apply_torque(0.0f);
			continue;
		}

		float torque = knob_haptic_update(&knob, status.sum_angle, status.rpm,
						  k_uptime_get_32());

		knob_apply_torque(torque);
	}
}

K_THREAD_DEFINE(knob_control, 1536, knob_control_thread, NULL, NULL, NULL, 3, 0, 0);

/**
 * @brief 手感状态监控线程
 */
void motor_monitor_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1) {
		k_msleep(1000);

		motor_status_t status;
		int ret = motor_get(m3508_motor, &status);

		if (ret != 0) {
			LOG_ERR("读取电机状态失败: %d", ret);
			continue;
		}

		LOG_INF("档位 %d | 角度 %.1f° | 转速 %.1f rpm | 指令 %.2f N·m | 越档 %u 次 | 在线 %d",
			knob_haptic_detent(&knob), (double)status.sum_angle, (double)status.rpm,
			(double)knob.torque, (unsigned int)knob_haptic_crossings(&knob),
			status.online);
	}
}

K_THREAD_DEFINE(motor_monitor, 1024, motor_monitor_thread, NULL, NULL, NULL, 5, 0, 0);

/**
 * @brief 主函数
 */
int main(void)
{
	LOG_INF("=== DJI M3508 飞机旋钮虚拟档位手感 ===");
	LOG_INF("档位间距 %.0f°, 一圈 %d 档, 力矩限幅 %.2f N·m",
		(double)knob_params.detent_spacing_deg, (int)(360.0f / knob_params.detent_spacing_deg),
		(double)knob_params.max_torque);

	if (!device_is_ready(m3508_motor)) {
		LOG_ERR("M3508电机设备未就绪");
		return -ENODEV;
	}

	motor_control(m3508_motor, ENABLE_MOTOR);
	LOG_INF("电机已启用");

	/* 等待 C620 周期反馈，确认总线与供电正常 */
	motor_status_t status = {0};
	bool online = false;

	for (int i = 0; i < 200; i++) {
		if (motor_get(m3508_motor, &status) == 0 && status.online) {
			online = true;
			break;
		}
		k_msleep(10);
	}

	if (!online) {
		LOG_ERR("未收到 M3508 周期反馈，请检查 CAN 接线、电机 ID 与 24V 供电");
		motor_control(m3508_motor, DISABLE_MOTOR);
		return -ENODEV;
	}

	/* 以当前位置为档位 0，便于观察档位编号 */
	motor_control(m3508_motor, SET_ZERO);
	k_msleep(20);

	motor_get(m3508_motor, &status);
	knob_haptic_init(&knob, &knob_params, status.sum_angle, k_uptime_get_32());
	knob_ready = true;

	LOG_INF("手感已启动，转动旋钮即可感受虚拟档位");

	return 0;
}
