#include "safety_state.hpp"

#include <cstdlib>
#include <iostream>

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

safety::Input healthyInput()
{
    safety::Input input {};
    input.command_ready = true;
    input.imu_online = true;
    input.acceleration_valid = true;
    input.all_wheels_online = true;
    input.velocity_estimate_valid = true;
    input.control_chain_valid = true;
    input.can_bus_healthy = true;
    input.board_link_online = true;
    input.referee_online = true;
    input.supercap_available = true;
    return input;
}

void testInitializationAndTransientRecovery()
{
    safety::StateMachine machine;
    machine.init(0U);

    safety::Output output = machine.update(healthyInput(), 0U);
    require(output.state == safety::State::Initializing,
            "initial healthy sample must still hold output");
    output = machine.update(healthyInput(), 249U);
    require(output.state == safety::State::Initializing,
            "must wait for initialization stability");
    require(!output.chassis_output_allowed,
            "initialization must not allow output");

    output = machine.update(healthyInput(), 250U);
    require(output.state == safety::State::Active,
            "healthy input must enter Active");
    require(output.chassis_output_allowed, "Active must allow output");

    safety::Input offline = healthyInput();
    offline.all_wheels_online = false;
    output = machine.update(offline, 1000U);
    require(output.state == safety::State::Standby,
            "wheel timeout must enter Standby");
    require(!output.chassis_output_allowed, "Standby must block output");

    output = machine.update(healthyInput(), 1100U);
    require(output.state == safety::State::Recovering,
            "cleared transient fault must enter Recovering");
    require(!output.chassis_output_allowed,
            "Recovering must block output");

    output = machine.update(healthyInput(), 1350U);
    require(output.state == safety::State::Active,
            "stable recovery must return to Active");
}

void testLatchedFaultAndReset()
{
    safety::StateMachine machine;
    machine.init(0U);
    (void)machine.update(healthyInput(), 0U);
    (void)machine.update(healthyInput(), 250U);

    safety::Input fault = healthyInput();
    fault.wheel_driver_fault_mask = 0x02U;
    safety::Output output = machine.update(fault, 500U);
    require(output.state == safety::State::FaultLatched,
            "driver fault must latch");
    require((output.latched_faults & safety::fault::kWheelDriverFault) != 0U,
            "driver fault reason must be reported");

    output = machine.update(healthyInput(), 600U);
    require(output.state == safety::State::FaultLatched,
            "latched fault must not clear automatically");
    const bool reset_accepted = machine.requestReset(600U);
    require(reset_accepted,
            "reset must be accepted after hardware fault clears");

    output = machine.update(healthyInput(), 601U);
    require(output.state == safety::State::Recovering,
            "reset must enter Recovering");
}

void testDisabledModeStartsInStandby()
{
    safety::StateMachine machine;
    machine.init(0U);

    safety::Input no_command = healthyInput();
    no_command.command_ready = false;
    safety::Output output = machine.update(no_command, 0U);
    require(output.state == safety::State::Initializing,
            "sensor initialization must run without a motion command");
    output = machine.update(no_command, 250U);
    require(output.state == safety::State::Standby,
            "disabled mode must enter Standby after initialization");
    require(!output.chassis_output_allowed,
            "disabled mode must block chassis output");
}

void testTemperatureAndModeDegradation()
{
    safety::StateMachine machine;
    machine.init(0U);
    (void)machine.update(healthyInput(), 0U);
    (void)machine.update(healthyInput(), 250U);

    safety::Input warning = healthyInput();
    warning.wheel_temperature_warning_mask = 0x01U;
    safety::Output output = machine.update(warning, 500U);
    require(output.state == safety::State::Degraded,
            "temperature warning must degrade control");
    require(output.chassis_output_allowed, "warning must keep output allowed");
    require(output.command_scale < 0.7f,
            "temperature warning must derate command");

    safety::Input critical = healthyInput();
    critical.wheel_temperature_critical_mask = 0x01U;
    critical.wheel_temperature_warning_mask = 0x01U;
    output = machine.update(critical, 600U);
    require(output.state == safety::State::Standby,
            "critical temperature must block output");

    output = machine.update(healthyInput(), 2100U);
    require(output.state == safety::State::Standby,
            "temperature clear dwell must start only after warning clears");
    output = machine.update(healthyInput(), 3600U);
    require(output.state == safety::State::Recovering,
            "temperature must remain blocked until clear dwell expires");
    output = machine.update(healthyInput(), 3850U);
    require(output.state == safety::State::Active,
            "temperature recovery must finish after stable dwell");
}

void testModeSpecificImuRequirements()
{
    safety::StateMachine machine;
    machine.init(0U);
    (void)machine.update(healthyInput(), 0U);
    (void)machine.update(healthyInput(), 250U);

    safety::Output output {};
    safety::Input wheel_only_auto = healthyInput();
    wheel_only_auto.auto_mode = true;
    wheel_only_auto.imu_online = false;
    wheel_only_auto.acceleration_valid = false;
    output = machine.update(wheel_only_auto, 3000U);
    require(output.state == safety::State::Standby,
            "auto mode without IMU must be blocked");

    safety::Input auto_without_acceleration = healthyInput();
    auto_without_acceleration.auto_mode = true;
    auto_without_acceleration.acceleration_valid = false;
    output = machine.update(auto_without_acceleration, 3050U);
    require(output.state == safety::State::Standby,
            "auto mode without acceleration must be blocked");

    safety::Input wheel_only_direct = healthyInput();
    wheel_only_direct.imu_online = false;
    wheel_only_direct.acceleration_valid = false;
    output = machine.update(wheel_only_direct, 3100U);
    require(output.state == safety::State::Recovering,
            "switching from blocking fault must enter Recovering");
    output = machine.update(wheel_only_direct, 3350U);
    require(output.chassis_output_allowed,
            "direct mode may run without IMU");
    require(output.command_scale < 0.85f,
            "wheel-only mode must derate command");
}

} // namespace

int main()
{
    testInitializationAndTransientRecovery();
    testLatchedFaultAndReset();
    testDisabledModeStartsInStandby();
    testTemperatureAndModeDegradation();
    testModeSpecificImuRequirements();
    std::cout << "safety_state_test: OK\n";
    return 0;
}
