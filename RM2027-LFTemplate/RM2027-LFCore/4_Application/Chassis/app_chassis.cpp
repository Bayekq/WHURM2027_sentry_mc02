/**
 * @file app_chassis.cpp
 * @brief 四轮全向底盘应用编排。
 *
 * 本文件只负责初始化和单周期调度。车型参数、命令选择、功率仲裁以及
 * 反馈/控制算法分别位于 chassis_config、chassis_command、
 * chassis_power_manager 和 chassis_controller。
 */

#include "app_chassis.h"

#include "app_board_link.hpp"
#include "app_supercap.h"
#include "bsp_can.h"
#include "chassis_command.hpp"
#include "chassis_config.hpp"
#include "chassis_controller.hpp"
#include "chassis_power_manager.hpp"
#include "cmsis_os.h"
#include "detect.h"
#include "dvc_buzzer.hpp"
#include "fdcan.h"
#include "lf_core.hpp"
#include "main.h"
#include "referee.hpp"
#include "remote_control.h"
#include "safety_state.hpp"

namespace {

//反馈回调函数
void publishChassisVelocity(const chassis_app::ChassisFeedback& feedback)
{
    LuojiaFox::system_state.updateChassis({
        feedback.vx,
        feedback.vy,
        feedback.wz
    });
}

uint8_t wheelDetectErrorMask(uint32_t detect_error_mask)
{
    uint8_t errors = 0U;
    for (uint8_t i = 0U; i < chassis_app::kWheelCount; ++i) {
        if ((detect_error_mask
             & (1UL << (DETECT_WHEEL1 + i))) != 0U) {
            errors |= static_cast<uint8_t>(1U << i);
        }
    }
    return errors;
}

void updateDetectHooks(
    const LuojiaFox::SystemStateSnapshot& state,
    const chassis_app::ChassisFeedback& feedback,
    bool rc_online,
    uint32_t now_ms
)
{
    if (rc_online) {
        detect_hook(DETECT_RC);
    }

    for (uint8_t i = 0U; i < chassis_app::kWheelCount; ++i) {
        if ((feedback.valid_wheel_mask
             & static_cast<uint8_t>(1U << i)) != 0U) {
            detect_hook(static_cast<uint8_t>(DETECT_WHEEL1 + i));
        }
    }

    const bool imu_fresh =
        state.imu.last_update_ms != 0U
        && now_ms - state.imu.last_update_ms
            <= chassis_app::kImuDataTimeoutMs;
    if (imu_fresh) {
        detect_hook(DETECT_BMI_GYRO);
        if (state.imu.acceleration_valid) {
            detect_hook(DETECT_BMI_ACCEL);
        }
    }
}

safety::Input buildSafetyInput(
    const LuojiaFox::SystemStateSnapshot& state,
    const chassis_app::ChassisCommandSelection& selection,
    const chassis_app::ChassisFeedback& feedback,
    uint32_t now_ms
)
{
    const uint32_t detect_error_mask = detectGetErrorMask();
    const uint8_t wheel_detect_errors =
        wheelDetectErrorMask(detect_error_mask);
    const bool imu_online =
        (detect_error_mask
         & ((1UL << DETECT_BMI_GYRO) | (1UL << DETECT_BMI_ACCEL))) == 0U;
    const bool rc_online =
        (detect_error_mask & (1UL << DETECT_RC)) == 0U;

    BspFdcanStats can_stats {};
    const bool can_bus_healthy =
        bspFdcanGetStats(&hfdcan1, &can_stats)
        && !can_stats.bus_off
        && !can_stats.recovering;

    const RefereePowerSnapshot referee_power =
        referee.GetPowerSupplySnapshot();
    const SupercapAppStatus supercap_status = supercapGetStatus();

    safety::Input input {};
    input.command_ready =
        selection.command.ready
        && (selection.mode != CHASSIS_MODE_DIRECT_RC || rc_online);
    input.auto_mode = selection.mode == CHASSIS_MODE_AUTO_VISION;
    input.imu_online = imu_online;
    input.acceleration_valid =
        imu_online && state.imu.acceleration_valid;
    input.all_wheels_online =
        feedback.all_wheels_online && wheel_detect_errors == 0U;
    input.velocity_estimate_valid = feedback.velocity_estimate_valid;
    input.control_chain_valid =
        chassis_app::g_chassisController.forceReady()
        && chassis_app::g_chassisPowerManager.enabled();
    input.can_bus_healthy = can_bus_healthy;
    input.board_link_online = board_link::isPeerOnline(now_ms);
    input.referee_online = referee_power.online;
    input.supercap_available =
        supercap_status == SUPERCAP_READY
        || supercap_status == SUPERCAP_NOT_CONFIGURED;
    input.wheel_driver_fault_mask = feedback.driver_fault_mask;
    input.wheel_temperature_warning_mask =
        feedback.temperature_warning_mask;
    input.wheel_temperature_critical_mask =
        feedback.temperature_critical_mask;
    return input;
}

} // namespace

void Chassis_Init(void)
{
    chassis_app::g_chassisCommandSelector =
        chassis_app::ChassisCommandSelector();
    chassis_app::g_chassisPowerManager.init();
    safety::g_safety.init(HAL_GetTick());

    if (!chassis_app::g_chassisController.init()) {
        Error_Handler();
    }
}

void Chassis_Control(void)
{
    const LuojiaFox::SystemStateSnapshot state =
        LuojiaFox::system_state.read();
    const uint32_t now_ms = HAL_GetTick();
    const bool rc_online = !getRcOffline();

    const chassis_app::ChassisCommandSelection selection =
        chassis_app::g_chassisCommandSelector.update(
            state.robot_cmd,
            rc_online,
            now_ms
        );
    buzzer.update(now_ms);
    const chassis_app::ChassisFeedback feedback =
        chassis_app::g_chassisController.readFeedback(state.imu, now_ms);
    publishChassisVelocity(feedback);
    updateDetectHooks(state, feedback, rc_online, now_ms);
    const safety::Output safety_output = safety::g_safety.update(
        buildSafetyInput(state, selection, feedback, now_ms),
        now_ms
    );

    // 模式切换后的首周期先停车并清除积分，避免继承上一控制源的历史状态。
    // 模式切换时通过蜂鸣器提示。
    if (selection.mode_changed) {
        chassis_app::g_chassisController.stop();
        switch (selection.mode) {
        case CHASSIS_MODE_DISABLED:
            buzzer.playBeeps(1U);
            break;
        case CHASSIS_MODE_DIRECT_RC:
            buzzer.playBeeps(2U);
            break;
        case CHASSIS_MODE_AUTO_VISION:
            buzzer.playBeeps(3U);
            break;
        default:
            break;
        }
        return;
    }

    if (safety_output.state_changed
        && safety_output.state == safety::State::FaultLatched) {
        buzzer.playBeeps(5U);
    }

    const bool auto_mode_allowed =
        selection.mode != CHASSIS_MODE_AUTO_VISION
        || safety_output.auto_mode_allowed;
    if (!safety_output.chassis_output_allowed || !auto_mode_allowed) {
        chassis_app::g_chassisController.stop();
        return;
    }

    float target_current[chassis_app::kWheelCount] = {};
    if (!chassis_app::g_chassisController.calculate(
            selection.command,
            feedback,
            target_current
        )) {
        chassis_app::g_chassisController.sendZero();
        return;
    }

    for (uint8_t i = 0U; i < chassis_app::kWheelCount; ++i) {
        target_current[i] *= safety_output.command_scale;
    }

    float limited_current[chassis_app::kWheelCount] = {};
    if (!chassis_app::g_chassisPowerManager.apply(
            feedback.wheel_omega,
            target_current,
            limited_current
        )) {
        chassis_app::g_chassisController.stop();
        return;
    }

    chassis_app::g_chassisController.sendCurrents(limited_current);
}

ChassisModeTypedef Chassis_GetMode(void)
{
    return chassis_app::g_chassisCommandSelector.mode();
}

#if BOARD_ROLE == 1
extern "C" void Control_Task(void const *argument)
{
    (void)argument;
    for (;;) {
        Chassis_Control();
        osDelay(1);
    }
}
#endif
