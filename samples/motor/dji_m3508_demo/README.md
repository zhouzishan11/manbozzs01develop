# DJI M3508 飞机旋钮虚拟档位手感示例

用 M3508 驱动一个没有机械档位的旋钮，通过软件生成“一档一档”的力矩反馈：
虚拟弹簧把旋钮拉向档位中心，阻尼抑制快速旋转时的振荡，越档瞬间叠加一个
短促的反向脉冲形成“咔哒”顿挫感。

## 功能特性

- **周期性虚拟档位**：档位中心等间距分布，弹簧力矩始终指向最近档位中心
- **越档检测**：基于多圈累计角度，带回滞，快速旋转一个周期可跨多档
- **越档顿挫脉冲**：方向与转动相反，按指数衰减，峰值与衰减时间可配
- **阻尼力矩**：与输出轴转速反向，转速经低通滤波以抑制 CAN 量化噪声
- **限幅保护**：力矩限幅、力矩变化率限幅、超速切断、离线自动归零
- **1 kHz 控制**：与 C620 的 1 kHz 周期反馈匹配，避免低控制频率引起抖动
- **模块化**：手感参数集中在 `struct knob_feel_params`，便于扩展其他旋钮手感

## 硬件与接口限制

| 项目 | 说明 |
| --- | --- |
| 力矩控制 | M3508 + C620 支持**电流（力矩）指令**，驱动按 `gear_ratio` 与 `TORQUE2CURRENT` 换算成电流值下发 |
| 力矩反馈 | **没有力矩传感器**。`motor_status_t.torque` 是由反馈电流换算的估计值，不能做真正的力矩闭环 |
| 闭环层级 | 软件输出力矩指令 → 电调内部电流环执行，即“电流闭环”，不是“力矩闭环” |
| 角度来源 | 只有电机自身编码器，经 19.2:1 减速比换算到输出轴；齿隙与减速箱柔性会削弱档位脆感 |
| 温度 | 当前 DJI 驱动未填充 `temperature` 字段（恒为 0），因此**没有软件过温保护** |
| 力矩限幅默认值 | 驱动默认 `±99999`，本示例每周期显式写入 `±max_torque`，勿删除该限幅 |

## 手感参数

`src/main.c` 中的 `knob_params`：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `detent_spacing_deg` | 30° | 档位间距，默认一圈 12 档 |
| `spring_stiffness` | 0.025 N·m/deg | 档位边界处约 0.375 N·m |
| `damping` | 0.0015 N·m/rpm | 100 rpm 时约 0.15 N·m |
| `detent_impulse` | 0.45 N·m | 越档脉冲峰值，0 表示关闭 |
| `impulse_decay_ms` | 12 ms | 脉冲指数衰减时间常数 |
| `max_torque` | 1.2 N·m | 输出力矩硬限幅 |
| `torque_slew_rate` | 120 N·m/s | 力矩变化率限幅 |
| `velocity_lpf_alpha` | 0.85 | 转速低通系数 |
| `crossing_hysteresis_deg` | 1.0° | 越档回滞，抑制边界抖动 |
| `deadband_deg` | 0.2° | 档位中心死区 |

调参建议：

- **档位太软**：增大 `spring_stiffness`，但需保证 `stiffness * spacing / 2 < max_torque`
- **顿挫不明显**：增大 `detent_impulse` 或减小 `impulse_decay_ms`
- **咔哒后抖动**：减小 `spring_stiffness`、增大 `damping`、增大 `crossing_hysteresis_deg`
- **快速旋转发飘或振荡**：减小 `max_torque`/增大 `damping`，必要时降低 `torque_slew_rate`

## 构建与烧录

在本仓库的 west 工作区根目录执行（`venv` 已激活）：

```bash
west build -b robomaster_board_c samples/motor/dji_m3508_demo --pristine
west flash --runner openocd
```

一键构建并烧录：

```bash
west build -b robomaster_board_c samples/motor/dji_m3508_demo --pristine && west flash --runner openocd
```

> 首次烧录若提示 `unable to open CMSIS-DAP device`，需安装 CMSIS-DAP 的 udev 规则并重新插拔调试器。

## 单元测试

手感算法与硬件无关，可在 `native_sim` 上验证角度回绕、档位切换、快速旋转与限幅：

```bash
west twister -T tests/native_sim/knob_haptic --platform native_sim/native/64 --inline-logs
```

## 硬件测试方案

前置条件：M3508 已可靠固定，旋钮与电机输出轴刚性连接，24V 供电与 CAN 接线正常。

1. **上电自检**
   - 串口应打印 `=== DJI M3508 飞机旋钮虚拟档位手感 ===` 和 `手感已启动`
   - 若打印 `未收到 M3508 周期反馈`，先检查 CAN_H/CAN_L、电机 ID、供电，不要继续加力测试
2. **静态弹簧**
   - 断电后手动把旋钮转到两档之间再松手，上电后应被拉回最近档位中心
   - 观察串口 `档位` 编号应与实际档位一致，角度应回到 `档位 × 30°`
3. **阻尼与限幅**
   - 手动快速旋转，应感到随转速增大的阻力，且不会出现持续自激振荡
   - 串口 `指令` 的绝对值不应超过 `max_torque`
4. **越档顿挫**
   - 慢速旋转，每越过一个档位应有一次明显的“咔哒”阻力脉冲
   - 在档位边界来回微动，`越档 N 次` 不应持续增长（回滞生效）
5. **快速旋转**
   - 快速连续旋转多圈，`越档` 计数应按档位数量累加，不应丢档或卡死
   - 转速超过 300 rpm 时输出力矩自动归零
6. **角度回绕**
   - 连续同方向旋转超过 ±360°，档位编号应连续递增/递减，力矩方向不应突变
7. **异常保护**
   - 拔掉 CAN 线：驱动超时后应停止输出电流，串口 `在线 0`
   - 重新插回后应自动恢复，无需重启

测试时请保持手能随时断开 24V 电源；出现异常振动或持续堵转立即断电。

## 代码结构

- `src/knob_haptic.h/.c`：与硬件无关的档位手感算法（纯浮点、可单测）
- `src/main.c`：电机初始化、1 kHz 控制线程、力矩下发与安全保护
- `tests/native_sim/knob_haptic/`：算法单元测试

扩展新旋钮手感时，新增一组 `struct knob_feel_params` 即可；如需不同力矩曲线
（如非线性弹簧、分段阻尼），在 `knob_haptic_update()` 的弹簧/阻尼计算处按
配置分支扩展。
