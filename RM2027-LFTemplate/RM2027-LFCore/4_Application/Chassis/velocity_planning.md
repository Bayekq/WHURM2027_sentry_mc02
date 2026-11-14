# 底盘速度规划

本文说明底盘速度、加速度和加加速度（Jerk）规划器的实现、接入方式、调试字段和实车标定过程。

## 1. 作用

遥控器、自动视觉和导航模块输出的是目标速度。如果目标速度在一个控制周期内从零跳到最大值，
PID、力分配和功率控制会立即请求很大的加速度和轮力矩。

速度规划器把不可实现的阶跃目标转换为连续目标：

```text
目标 vx/vy/wz
      ↓
加速度上限
      ↓
Jerk 上限
      ↓
可执行的 vx/vy/wz 和 ax/ay/alpha
```

规划器负责正常运动品质：

- 限制起步和制动加速度
- 限制加速度变化率
- 降低电流和功率峰值
- 减少换向冲击
- 为打滑控制提供更合理的轮速目标
- 为后续加速度前馈提供 `ax/ay/alpha`

安全故障急停不经过规划器。安全状态机禁止输出时，底盘必须立即发送零电流。

## 2. 控制位置

当前控制链：

```text
系统状态
  -> 命令选择
  -> 安全检查
  -> 运动目标补偿
  -> 速度规划
  -> vx/vy/wz PID
  -> 四轮力分配
  -> 功率限制
  -> CAN
```

规划器在 `ChassisController::calculate()` 中执行，位于运动目标补偿之后、PID 之前。

相关文件：

| 文件 | 职责 |
|---|---|
| `velocity_planner.hpp` | 参数、规划输出和规划器接口 |
| `velocity_planner.cpp` | 加速度、Jerk、二维合成限制和目标吸合 |
| `chassis_config.hpp` | 实车可调参数 |
| `chassis_controller.cpp/.hpp` | 规划器实例、初始化和 PID 接入 |
| [velocity_planner_test.cpp](../../../tests/velocity_planner_test.cpp) | 主机规划测试 |

## 3. 规划状态

规划器保存：

```text
规划速度：vx, vy, wz
规划加速度：ax, ay, alpha
```

每个控制周期执行：

1. 计算目标速度误差。
2. 根据误差和当前加速度计算期望加速度。
3. 对期望加速度施加二维合成加速度上限。
4. 对加速度变化量施加二维 Jerk 上限。
5. 积分得到新的规划速度。
6. 当速度和加速度都接近目标时吸合到目标。

基本关系：

```text
error = target_velocity - planned_velocity

desired_acceleration =
    tracking_gain * error
    - damping_gain * planned_acceleration

delta_acceleration <= jerk_limit * dt

next_velocity =
    planned_velocity + acceleration * dt
```

阻尼项用于避免速度接近目标时因为加速度滞后而持续振荡。

## 4. vx/vy 二维限制

`vx` 和 `vy` 分别限制会导致斜向运动时合成加速度达到：

```text
sqrt(ax_max² + ay_max²)
```

因此规划器同时限制二维向量：

```text
sqrt(ax² + ay²) <= linear_accel_limit

sqrt(delta_ax² + delta_ay²) <= linear_jerk_limit * dt
```

加速度和 Jerk 超限时保持向量方向并统一缩放，不改变整车期望运动方向。

## 5. Yaw 限制

`wz` 使用独立参数：

- `yaw_accel_rad_s2`
- `yaw_decel_rad_s2`
- `yaw_jerk_rad_s3`

Yaw 不作为二维线速度向量的一部分，因为角速度和线速度单位不同。

## 6. 参数

参数位于 `chassis_config.hpp`：

| 参数 | 当前值 | 单位 | 含义 |
|---|---:|---|---|
| `kEnableVelocityPlanner` | `true` | - | 是否启用规划器 |
| `kPlannerLinearAccelMps2` | 1.5 | m/s² | 线速度加速上限 |
| `kPlannerLinearDecelMps2` | 2.0 | m/s² | 线速度减速上限 |
| `kPlannerLinearJerkMps3` | 12.0 | m/s³ | 线速度 Jerk 上限 |
| `kPlannerYawAccelRadS2` | 6.0 | rad/s² | Yaw 加速上限 |
| `kPlannerYawDecelRadS2` | 8.0 | rad/s² | Yaw 减速上限 |
| `kPlannerYawJerkRadS3` | 60.0 | rad/s³ | Yaw Jerk 上限 |
| `kPlannerTrackingGainPerS` | 10.0 | 1/s | 速度误差到期望加速度的增益 |
| `kPlannerDampingGainPerS` | 6.0 | 1/s | 加速度阻尼 |
| `kPlannerVelocityEpsilon` | 0.002 | m/s 或 rad/s | 目标吸合死区 |
| `kPlannerMinimumDtS` | 0.0005 | s | 最小合法控制周期 |
| `kPlannerMaximumDtS` | 0.0025 | s | 最大合法控制周期 |

当前参数是保守初值，不是最终实车标定结果。

## 7. 初始化、复位和安全边界

`ChassisController::init()` 初始化规划器。初始化失败时底盘初始化失败。

以下情况会调用 `velocity_planner_.reset()`：

- 安全状态禁止输出
- 模式切换
- 控制链异常
- 速度反馈不可用
- 力分配失败
- 重新使能

复位后规划速度和规划加速度都为零，避免重新进入闭环时继承旧目标。

规划器不能代替：

- PID：负责跟随规划目标
- 打滑检测：负责处理实际地面附着不足
- 功率控制：负责裁判和超级电容功率限制
- 安全状态机：负责故障锁存和立即零输出

## 8. 调试

在 Live Watch 中展开：

```text
g_chassisController.latest_plan
```

关键字段：

| 字段 | 含义 |
|---|---|
| `vx` | 规划后的 x 向速度，m/s |
| `vy` | 规划后的 y 向速度，m/s |
| `wz` | 规划后的旋转角速度，rad/s |
| `ax` | 当前规划加速度，m/s² |
| `ay` | 当前规划加速度，m/s² |
| `alpha` | 当前规划 Yaw 角加速度，rad/s² |

同时观察：

```text
selection.command
g_chassisController.latest_plan
g_chassisController.latest_feedback
g_chassisController.motionAccelerationX()
g_chassisController.motionAccelerationY()
g_chassisPowerManager
g_safety
```

建议同时记录：

- 原始目标速度
- 规划速度
- 规划加速度
- 实测轮速和底盘速度
- 四轮电流
- 功率模型估计
- 打滑分数
- 安全状态和故障位

## 9. 主机测试

从 `RM2027-LFTemplate` 目录执行：

```powershell
g++ -std=c++14 -Wall -Wextra -Wpedantic `
  -I RM2027-LFCore/4_Application/Chassis `
  tests/velocity_planner_test.cpp `
  RM2027-LFCore/4_Application/Chassis/velocity_planner.cpp `
  -o build/velocity_planner_test.exe

.\build\velocity_planner_test.exe
```

测试覆盖：

- 无效参数拒绝
- 阶跃目标的速度响应
- 线加速度上限
- 线 Jerk 上限
- 二维合成加速度限制
- 目标无超调吸合
- 前进到后退换向
- Yaw 角加速度限制
- 非法控制周期
- 手动复位

## 10. 实车标定

建议按以下顺序：

1. 先只记录原始目标和规划目标，确认输入方向和单位正确。
2. 关闭自动视觉，使用遥控小步阶跃测试。
3. 逐步提高 `kPlannerLinearAccelMps2`，观察轮速、打滑分数和电流峰值。
4. 将正常测得的最大可用加速度保留 20%～30% 安全余量。
5. 分别标定加速和减速，不要假设两者相同。
6. 调整 Jerk，直到起步和换向没有明显冲击，电流不出现尖峰。
7. 单独标定 `wz`，避免原地旋转造成轮子打滑。
8. 调整 `tracking_gain` 和 `damping_gain`，保证接近目标时不振荡。
9. 进行斜向和连续换向测试，确认二维合成限制有效。
10. 最后接入自动视觉，检查目标跳变和通信超时行为。

## 11. 已知边界

- 当前使用固定 `kControlPeriodS = 0.001f`，与现有 PID 周期一致。
- 规划结果最终仍受电流、功率、附着和打滑控制限制。
- 安全状态机禁止输出时不会经过规划器平滑停车，而是立即零输出。
- 规划目标吸合使用速度死区；死区越大，稳态速度误差越大，但末端更容易稳定。
- 当前参数尚未经过实车验证，首次测试必须架空或确保急停可用。
