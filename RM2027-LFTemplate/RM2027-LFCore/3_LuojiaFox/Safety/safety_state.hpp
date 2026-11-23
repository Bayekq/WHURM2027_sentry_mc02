/**
 * @file safety_state.hpp
 * @brief 整车安全状态机与输出许可。
 */

#ifndef SAFETY_STATE_HPP
#define SAFETY_STATE_HPP

#include <stdint.h>

namespace safety {

/** @brief 安全状态机状态。 */
enum class State : uint8_t {
    Initializing = 0,
    Standby,
    Active,
    Degraded,
    Recovering,
    FaultLatched,
};

/** @brief 故障与告警位。 */
namespace fault {
constexpr uint32_t kNone = 0U;
constexpr uint32_t kCommandUnavailable = 1U << 0U;
constexpr uint32_t kImuUnavailable = 1U << 1U;
constexpr uint32_t kWheelOffline = 1U << 2U;
constexpr uint32_t kWheelDriverFault = 1U << 3U;
constexpr uint32_t kWheelOvertemperature = 1U << 4U;
constexpr uint32_t kWheelTemperatureWarning = 1U << 5U;
constexpr uint32_t kVelocityEstimateInvalid = 1U << 6U;
constexpr uint32_t kControlChainInvalid = 1U << 7U;
constexpr uint32_t kCanBusFault = 1U << 8U;
constexpr uint32_t kEmergencyStop = 1U << 9U;
constexpr uint32_t kBoardLinkOffline = 1U << 10U;
constexpr uint32_t kRefereeOffline = 1U << 11U;
constexpr uint32_t kSupercapUnavailable = 1U << 12U;
constexpr uint32_t kAccelerationUnavailable = 1U << 13U;
} // namespace fault

/** @brief 安全状态机输入快照。 */
struct Input {
    bool emergency_stop = false;
    bool command_ready = false;
    bool auto_mode = false;
    bool imu_online = false;
    bool acceleration_valid = false;
    bool all_wheels_online = false;
    bool velocity_estimate_valid = false;
    bool control_chain_valid = false;
    bool can_bus_healthy = false;
    bool board_link_online = false;
    bool referee_online = false;
    bool supercap_available = false;
    uint8_t wheel_driver_fault_mask = 0U;
    uint8_t wheel_temperature_warning_mask = 0U;
    uint8_t wheel_temperature_critical_mask = 0U;
};

/** @brief 供底盘和调试器读取的安全输出快照。 */
struct Output {
    State state = State::Initializing;
    uint32_t active_faults = fault::kNone;
    uint32_t latched_faults = fault::kNone;
    uint32_t warnings = fault::kNone;
    bool chassis_output_allowed = false;
    bool auto_mode_allowed = false;
    bool state_changed = false;
    float command_scale = 0.0f;
    uint32_t state_since_ms = 0U;
};

/**
 * @brief 将设备健康检测转换为整车安全决策。
 * @note 该类不访问 HAL，可在主机侧注入输入进行单元测试。
 */
class StateMachine {
public:
    /** @brief 上电初始化，进入 Initializing 状态。 */
    void init(uint32_t now_ms);

    /**
     * @brief 更新状态机。
     * @param input 本周期健康、命令和故障输入。
     * @param now_ms 当前系统时间。
     * @return 本周期输出许可和调试快照。
     */
    Output update(const Input& input, uint32_t now_ms);

    /**
     * @brief 请求清除锁存故障。
     * @return 当前硬件故障已消失且允许复位时返回 true。
     * @note 急停、电机驱动故障和 CAN Bus-off 不会因为简单复位请求而
     *       忽略仍然存在的硬件故障。
     */
    bool requestReset(uint32_t now_ms);

    /** @brief 获取最近一次输出快照。 */
    Output snapshot() const
    {
        return output_;
    }

private:
    bool canClearLatchedFaults() const;
    void updateThermalBlock(const Input& input, uint32_t now_ms);
    void changeState(State state, uint32_t now_ms);

    State state_ = State::Initializing;
    uint32_t state_since_ms_ = 0U;
    uint32_t stable_since_ms_ = 0U;
    uint32_t thermal_clear_since_ms_ = 0U;
    uint32_t latched_faults_ = fault::kNone;
    bool stable_timer_active_ = false;
    bool thermal_clear_timer_active_ = false;
    bool thermal_block_active_ = false;
    Input last_input_ {};
    Output output_ {};
};

extern StateMachine g_safety;

} // namespace safety

#endif // SAFETY_STATE_HPP
