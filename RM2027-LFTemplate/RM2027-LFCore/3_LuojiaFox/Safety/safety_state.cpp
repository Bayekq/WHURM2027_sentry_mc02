/**
 * @file safety_state.cpp
 * @brief 整车安全状态机实现。
 */

//核心观察变量
// g_safety.state
// g_safety.active_faults
// g_safety.latched_faults
// g_safety.warnings
// g_safety.chassis_output_allowed
// g_safety.auto_mode_allowed
// g_safety.command_scale
// g_safety.state_since_ms
// g_safety.state_changed




#include "safety_state.hpp"

#include <algorithm>

namespace safety {

namespace {

constexpr uint32_t kInitializationStableMs = 250U;
constexpr uint32_t kRecoveryStableMs = 250U;
constexpr uint32_t kThermalClearStableMs = 1500U;
constexpr float kThermalCommandScale = 0.65f;
constexpr float kWheelOnlyCommandScale = 0.80f;

} // namespace

StateMachine g_safety;

void StateMachine::changeState(State state, uint32_t now_ms)
{
    if (state_ == state) {
        return;
    }
    state_ = state;
    state_since_ms_ = now_ms;
}

void StateMachine::init(uint32_t now_ms)
{
    state_ = State::Initializing;
    state_since_ms_ = now_ms;
    stable_since_ms_ = 0U;
    thermal_clear_since_ms_ = 0U;
    stable_timer_active_ = false;
    thermal_clear_timer_active_ = false;
    latched_faults_ = fault::kNone;
    thermal_block_active_ = false;
    last_input_ = {};
    output_ = {};
    output_.state = state_;
    output_.state_since_ms = now_ms;
}

void StateMachine::updateThermalBlock(const Input& input, uint32_t now_ms)
{
    if (input.wheel_temperature_critical_mask != 0U) {
        thermal_block_active_ = true;
        thermal_clear_since_ms_ = 0U;
        thermal_clear_timer_active_ = false;
        return;
    }

    if (!thermal_block_active_) {
        thermal_clear_since_ms_ = 0U;
        thermal_clear_timer_active_ = false;
        return;
    }

    // 必须先回到告警阈值以下，并持续稳定，才允许退出过温封锁。
    if (input.wheel_temperature_warning_mask != 0U) {
        thermal_clear_since_ms_ = 0U;
        thermal_clear_timer_active_ = false;
        return;
    }

    if (!thermal_clear_timer_active_) {
        thermal_clear_timer_active_ = true;
        thermal_clear_since_ms_ = now_ms;
    }
    if (now_ms - thermal_clear_since_ms_ >= kThermalClearStableMs) {
        thermal_block_active_ = false;
        thermal_clear_since_ms_ = 0U;
        thermal_clear_timer_active_ = false;
    }
}

bool StateMachine::canClearLatchedFaults() const
{
    return !last_input_.emergency_stop
        && last_input_.wheel_driver_fault_mask == 0U
        && last_input_.wheel_temperature_warning_mask == 0U
        && last_input_.wheel_temperature_critical_mask == 0U
        && last_input_.can_bus_healthy;
}

bool StateMachine::requestReset(uint32_t now_ms)
{
    if (!canClearLatchedFaults()) {
        return false;
    }

    latched_faults_ = fault::kNone;
    thermal_block_active_ = false;
    thermal_clear_since_ms_ = 0U;
    thermal_clear_timer_active_ = false;
    stable_since_ms_ = 0U;
    stable_timer_active_ = false;
    changeState(State::Standby, now_ms);
    output_.latched_faults = fault::kNone;
    output_.state = state_;
    output_.state_since_ms = state_since_ms_;
    output_.chassis_output_allowed = false;
    output_.auto_mode_allowed = false;
    output_.command_scale = 0.0f;
    return true;
}

Output StateMachine::update(const Input& input, uint32_t now_ms)
{
    const State previous_state = state_;
    last_input_ = input;
    updateThermalBlock(input, now_ms);

    uint32_t latched_now = fault::kNone;
    if (input.emergency_stop) {
        latched_now |= fault::kEmergencyStop;
    }
    if (input.wheel_driver_fault_mask != 0U) {
        latched_now |= fault::kWheelDriverFault;
    }
    if (!input.can_bus_healthy) {
        latched_now |= fault::kCanBusFault;
    }
    latched_faults_ |= latched_now;

    uint32_t blocking = fault::kNone;
    uint32_t warnings = fault::kNone;

    if (!input.command_ready) {
        blocking |= fault::kCommandUnavailable;
    }
    if (!input.all_wheels_online) {
        blocking |= fault::kWheelOffline;
    }
    if (!input.control_chain_valid) {
        blocking |= fault::kControlChainInvalid;
    }
    if (!input.velocity_estimate_valid) {
        blocking |= fault::kVelocityEstimateInvalid;
    }
    if (!input.imu_online) {
        if (input.auto_mode) {
            blocking |= fault::kImuUnavailable;
        } else {
            warnings |= fault::kImuUnavailable;
        }
    }
    if (!input.acceleration_valid) {
        if (input.auto_mode) {
            blocking |= fault::kAccelerationUnavailable;
        } else {
            warnings |= fault::kAccelerationUnavailable;
        }
    }
    if (thermal_block_active_) {
        blocking |= fault::kWheelOvertemperature;
    }
    if (input.wheel_temperature_warning_mask != 0U) {
        warnings |= fault::kWheelTemperatureWarning;
    }
    if (!input.board_link_online) {
        warnings |= fault::kBoardLinkOffline;
    }
    if (!input.referee_online) {
        warnings |= fault::kRefereeOffline;
    }
    if (!input.supercap_available) {
        warnings |= fault::kSupercapUnavailable;
    }

    // 初始化只等待传感和控制链，不要求操作手已经给出运动命令。
    const bool core_healthy =
        ((blocking & ~fault::kCommandUnavailable) | latched_faults_)
        == fault::kNone;

    if (latched_faults_ != fault::kNone) {
        stable_since_ms_ = 0U;
        stable_timer_active_ = false;
        changeState(State::FaultLatched, now_ms);
    } else if (!core_healthy) {
        stable_since_ms_ = 0U;
        stable_timer_active_ = false;
        changeState(
            state_ == State::Initializing ? State::Initializing : State::Standby,
            now_ms
        );
    } else if (state_ == State::Initializing) {
        if (!stable_timer_active_) {
            stable_timer_active_ = true;
            stable_since_ms_ = now_ms;
        }
        if (now_ms - stable_since_ms_ >= kInitializationStableMs) {
            if (!input.command_ready) {
                changeState(State::Standby, now_ms);
            } else {
                changeState(
                    warnings == fault::kNone ? State::Active : State::Degraded,
                    now_ms
                );
            }
            stable_since_ms_ = 0U;
            stable_timer_active_ = false;
        }
    } else if (!input.command_ready) {
        stable_since_ms_ = 0U;
        stable_timer_active_ = false;
        changeState(State::Standby, now_ms);
    } else if (state_ == State::Standby || state_ == State::Recovering) {
        if (state_ != State::Recovering || !stable_timer_active_) {
            changeState(State::Recovering, now_ms);
            stable_since_ms_ = now_ms;
            stable_timer_active_ = true;
        }
        if (now_ms - stable_since_ms_ >= kRecoveryStableMs) {
            changeState(
                warnings == fault::kNone ? State::Active : State::Degraded,
                now_ms
            );
            stable_since_ms_ = 0U;
            stable_timer_active_ = false;
        }
    } else {
        stable_timer_active_ = false;
        changeState(
            warnings == fault::kNone ? State::Active : State::Degraded,
            now_ms
        );
    }

    const bool output_allowed =
        input.command_ready
        && (state_ == State::Active || state_ == State::Degraded);

    float command_scale = output_allowed ? 1.0f : 0.0f;
    if (input.wheel_temperature_warning_mask != 0U) {
        command_scale = std::min(command_scale, kThermalCommandScale);
    }
    if (!input.imu_online && !input.auto_mode) {
        command_scale = std::min(command_scale, kWheelOnlyCommandScale);
    }

    output_.state = state_;
    output_.active_faults = blocking | warnings | latched_faults_;
    output_.latched_faults = latched_faults_;
    output_.warnings = warnings;
    output_.chassis_output_allowed = output_allowed;
    output_.auto_mode_allowed =
        output_allowed && input.imu_online && input.acceleration_valid;
    output_.state_changed = previous_state != state_;
    output_.command_scale = command_scale;
    output_.state_since_ms = state_since_ms_;
    return output_;
}

} // namespace safety
