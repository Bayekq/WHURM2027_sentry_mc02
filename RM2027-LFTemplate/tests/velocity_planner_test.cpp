#include "velocity_planner.hpp"

#include <cmath>
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

bool near(float actual, float expected, float tolerance)
{
    return std::fabs(actual - expected) <= tolerance;
}

chassis_app::VelocityPlannerConfig testConfig()
{
    chassis_app::VelocityPlannerConfig config {};
    config.linear_accel_mps2 = 1.5f;
    config.linear_decel_mps2 = 2.0f;
    config.linear_jerk_mps3 = 12.0f;
    config.yaw_accel_rad_s2 = 6.0f;
    config.yaw_decel_rad_s2 = 8.0f;
    config.yaw_jerk_rad_s3 = 60.0f;
    config.tracking_gain_per_s = 10.0f;
    config.damping_gain_per_s = 6.0f;
    config.velocity_epsilon = 0.002f;
    config.minimum_dt_s = 0.0005f;
    config.maximum_dt_s = 0.0025f;
    return config;
}

void testConfigurationValidation()
{
    chassis_app::VelocityPlanner planner;
    chassis_app::VelocityPlannerConfig config = testConfig();
    config.linear_jerk_mps3 = 0.0f;
    require(!planner.init(config), "invalid jerk must be rejected");
}

void testLinearStepResponse()
{
    constexpr float dt = 0.001f;
    constexpr float target = 1.0f;
    const chassis_app::VelocityPlannerConfig config = testConfig();
    chassis_app::VelocityPlanner planner;
    require(planner.init(config), "planner initialization failed");

    chassis_app::VelocityPlan previous = planner.plan();
    for (int step = 0; step < 5000; ++step) {
        const chassis_app::VelocityPlan current =
            planner.update(target, 0.0f, 0.0f, dt);
        require(std::isfinite(current.vx), "planned velocity must be finite");
        require(std::fabs(current.ax) <= config.linear_accel_mps2 + 1.0e-4f,
                "linear acceleration limit exceeded");
        require(current.vx <= target + 1.0e-6f,
                "linear planner overshot its target");

        const float acceleration_step =
            std::fabs(current.ax - previous.ax);
        if (acceleration_step
            > config.linear_jerk_mps3 * dt + 1.0e-4f) {
            std::cerr << "step=" << step
                      << " previous_vx=" << previous.vx
                      << " vx=" << current.vx
                      << " previous_ax=" << previous.ax
                      << " ax=" << current.ax
                      << " delta_ax=" << acceleration_step
                      << '\n';
        }
        require(
            acceleration_step
                <= config.linear_jerk_mps3 * dt + 1.0e-4f,
            "linear jerk limit exceeded"
        );
        previous = current;
    }

    require(near(planner.plan().vx, target, config.velocity_epsilon),
            "linear speed did not settle at target");
    require(std::fabs(planner.plan().ax) < 1.0e-5f,
            "linear acceleration did not settle to zero");
}

void testTwoDimensionalLimit()
{
    constexpr float dt = 0.001f;
    const chassis_app::VelocityPlannerConfig config = testConfig();
    chassis_app::VelocityPlanner planner;
    require(planner.init(config), "planner initialization failed");

    for (int step = 0; step < 10000; ++step) {
        const chassis_app::VelocityPlan current =
            planner.update(1.0f, 1.0f, 0.0f, dt);
        const float acceleration_norm =
            std::sqrt(current.ax * current.ax + current.ay * current.ay);
        require(acceleration_norm <= config.linear_accel_mps2 + 1.0e-4f,
                "two-dimensional acceleration limit exceeded");
    }

    require(near(planner.plan().vx, 1.0f, config.velocity_epsilon),
            "x speed did not settle");
    require(near(planner.plan().vy, 1.0f, config.velocity_epsilon),
            "y speed did not settle");
}

void testReversalAndYaw()
{
    constexpr float dt = 0.001f;
    const chassis_app::VelocityPlannerConfig config = testConfig();
    chassis_app::VelocityPlanner planner;
    require(planner.init(config), "planner initialization failed");
    planner.reset(1.0f, 0.0f, 0.0f);

    for (int step = 0; step < 10000; ++step) {
        (void)planner.update(-1.0f, 0.0f, 2.0f, dt);
        require(std::isfinite(planner.plan().vx), "reversal velocity invalid");
        require(std::isfinite(planner.plan().wz), "yaw velocity invalid");
        require(std::fabs(planner.plan().alpha)
                    <= config.yaw_accel_rad_s2 + 1.0e-4f,
                "yaw acceleration limit exceeded");
    }

    if (!near(planner.plan().vx, -1.0f, config.velocity_epsilon)) {
        std::cerr << "reversal vx=" << planner.plan().vx
                  << " ax=" << planner.plan().ax
                  << " wz=" << planner.plan().wz
                  << " alpha=" << planner.plan().alpha
                  << '\n';
    }
    require(near(planner.plan().vx, -1.0f, config.velocity_epsilon),
            "reversal did not settle");
    require(near(planner.plan().wz, 2.0f, config.velocity_epsilon),
            "yaw target did not settle");
}

void testInvalidDtAndReset()
{
    chassis_app::VelocityPlanner planner;
    require(planner.init(testConfig()), "planner initialization failed");
    planner.reset(0.2f, -0.1f, 0.3f);
    const chassis_app::VelocityPlan plan =
        planner.update(0.5f, 0.0f, 0.0f, 0.0f);
    require(std::isfinite(plan.vx), "invalid dt must not produce NaN");
    planner.reset();
    require(planner.plan().vx == 0.0f, "reset must clear planned velocity");
    require(planner.plan().ax == 0.0f, "reset must clear planned acceleration");
}

} // namespace

int main()
{
    testConfigurationValidation();
    testLinearStepResponse();
    testTwoDimensionalLimit();
    testReversalAndYaw();
    testInvalidDtAndReset();
    std::cout << "velocity_planner_test: OK\n";
    return 0;
}
