# 四轮全向底盘应用

控制链路：

```text
系统状态
  -> 命令选择
  -> 安全检查
  -> 电机/IMU 反馈
  -> 速度目标补偿
  -> 速度/加速度/Jerk 规划
  -> vx/vy/wz 速度环
  -> 四轮力分配
  -> 功率衰减
  -> CAN 电流命令
```
## 遥控器模式
| `SC` 位置 | `rc_state.rc.s[1]` | 模式 | 蜂鸣 |
|---|---:|---|---|
| 上档 | `1` | `AUTO_VISION` 自动视觉 | 切入时响 3 声 |
| 中档 | `3` | `DIRECT_RC` 直接遥控 | 切入时响 2 声 |
| 下档 | `2` | `DISABLED` 禁止输出 | 切入时响 1 声 |
| `SB` 位置 | `rc_state.rc.s[0]` | 模式 | 蜂鸣 |
| 下档 | `2` | `DISABLED` 强制禁止输出 | 切入时响 1 声 |

## 文件职责

| 文件 | 职责 |
|---|---|
| `app_chassis.cpp` | 初始化、单周期调度和 FreeRTOS 任务入口 |
| `chassis_config.hpp` | 车型尺寸、电机配置、速度/力矩限制、PID 和超时参数 |
| `chassis_command.hpp/.cpp` | 模式选择、遥控/上位机命令、命令时效检查 |
| `chassis_controller.hpp/.cpp` | 反馈读取、轮速/IMU 融合、速度闭环、力分配和 CAN 发送 |
| `chassis_power_manager.hpp/.cpp` | 裁判系统/超级电容功率上限仲裁和电流衰减 |
| `velocity_planner.hpp/.cpp` | 正常运动目标的加速度、Jerk 和二维合成限制 |

`app_chassis.cpp` 不应重新增加算法细节。新的底层能力应放到对应算法模块，
车型差异应放入 `chassis_config.hpp`。

## 调试对象

可在调试器 Live Watch 中直接展开以下对象：

- `g_chassisCommandSelector`
  - `mode()`：当前底盘模式。
  - `sourceReady()`：当前控制源是否在线、新鲜且数值有效。
  - `sourceUpdateMs()`：当前控制源最近更新时间。

- `g_chassisController`
  - `forceReady()`：力分配矩阵是否构建成功。
  - `motionReady()`：轮速/IMU 融合器是否初始化成功。
  - `motionUsingAcceleration()`：本周期是否使用有效 IMU 加速度。
  - `motionAccelerationX/Y()`：融合后的底盘 x/y 向加速度。
  - `gravityReady()`：重力前馈器是否初始化成功。
  - `latest_plan`：规划后的 `vx/vy/wz` 和 `ax/ay/alpha`。
  - `latest_feedback`：逐轮打滑、重力投影和前馈力快照。
  - 私有成员可在调试器中继续展开，查看电机、PID、滤波器和力分配内部状态。

- `g_chassisPowerManager`
  - `effective_limit_w`：本次生效的功率上限。
  - `estimated_power_w`：功率衰减前模型总功率。
  - `consumed_power_w`：功率衰减后模型总功率。
  - `status`：功率控制状态。

## 当前控制边界

- 轮速与 IMU 加速度使用恒加速度卡尔曼滤波融合。
- 速度目标经过加速度和 Jerk 限制后再进入 PID。
- 目标速度补偿以 `K * |yaw_rate|` 为增益。
- `Slip_compensation1` 已基于轮速校正前的 IMU 预测速度提供逐轮打滑检测。
- 打滑置信度已用于轮速加权速度估计；当前尚未进入四轮力矩重分配。
- 坡道重力前馈已接入 `Fx/Fy` 广义力，但整车质量当前为零，实际补偿输出为零。
- `ForceSolver` 已接入，但轮级摩擦、惯性和关节重力模型参数当前为零；启用前必须完成实车标定。
- 云台与底盘相对 Yaw 当前尚未接入，融合器中的 `yaw_rad` 固定为 0。

## 安全与故障状态机

`RM2027-LFCore/3_LuojiaFox/Safety` 汇总 `Detect`、轮电机故障码、温度、
CAN Bus-off、裁判系统、超级电容和双板链路状态：

| 状态 | 行为 |
|---|---|
| `Initializing` | 等待 IMU、四轮反馈和控制链稳定，禁止输出 |
| `Active` | 全部核心条件正常，允许输出 |
| `Degraded` | 核心控制安全但存在告警，允许输出并限制自动模式或电流 |
| `Recovering` | 瞬态故障已消失，稳定观察期间仍禁止输出 |
| `Standby` | 命令、轮反馈、速度估计或控制链不可用，输出零电流 |
| `FaultLatched` | 急停、轮驱故障或 CAN Bus-off 已锁存，输出零电流 |

轮电机故障、Bus-off 和急停不会自动恢复，需要在硬件故障消失后调用
`safety::g_safety.requestReset()`。临界过温会先停车，待所有轮低于告警温度
并稳定 1.5 秒后自动恢复；75 ℃ 告警时输出电流缩放到 65%。

`Chassis_Control()` 每周期调用 `safety::g_safety.update()` 获取输出许可。
调试器可直接展开 `g_safety` 查看 `state`、`active_faults`、`latched_faults`、
`warnings` 和 `command_scale`。

完整输入、故障位、状态转换和扩展约定见
[安全与故障状态机文档](../../3_LuojiaFox/Safety/README.md)。

速度规划算法、参数和实车标定步骤见
[底盘速度规划文档](velocity_planning.md)。


