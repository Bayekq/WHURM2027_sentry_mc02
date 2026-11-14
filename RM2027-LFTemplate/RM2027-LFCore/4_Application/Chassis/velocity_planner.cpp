/**
 * @file velocity_planner.cpp
 * @brief 底盘速度、加速度和加加速度规划实现。
 */

#include "velocity_planner.hpp"

#include <cmath>

namespace chassis_app {

namespace {

/** @brief 将输入限制到闭区间 [lower, upper]。 */
float clampValue(float value, float lower, float upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

/** @brief 单周期内按最大变化量逼近目标。 */
float moveToward(float value, float target, float maximum_delta)
{
    if (target > value + maximum_delta) {
        return value + maximum_delta;
    }
    if (target < value - maximum_delta) {
        return value - maximum_delta;
    }
    return target;
}

/**
 * @brief 对二维向量的模长进行限幅，保持向量方向。
 * @return 输入向量模长超过上限时返回 true。
 */
void limitVector(float* x, float* y, float maximum_norm)
{
    const float norm_squared = (*x) * (*x) + (*y) * (*y);
    if (norm_squared <= maximum_norm * maximum_norm) {
        return;
    }

    const float norm = std::sqrt(norm_squared);
    if (norm <= 1.0e-6f) {
        *x = 0.0f;
        *y = 0.0f;
        return;
    }

    const float scale = maximum_norm / norm;
    *x *= scale;
    *y *= scale;
}

/** @brief 判断有限控制周期和规划参数是否有效。 */
bool configValid(const VelocityPlannerConfig& config)
{
    return std::isfinite(config.linear_accel_mps2)
        && config.linear_accel_mps2 > 0.0f
        && std::isfinite(config.linear_decel_mps2)
        && config.linear_decel_mps2 > 0.0f
        && std::isfinite(config.linear_jerk_mps3)
        && config.linear_jerk_mps3 > 0.0f
        && std::isfinite(config.yaw_accel_rad_s2)
        && config.yaw_accel_rad_s2 > 0.0f
        && std::isfinite(config.yaw_decel_rad_s2)
        && config.yaw_decel_rad_s2 > 0.0f
        && std::isfinite(config.yaw_jerk_rad_s3)
        && config.yaw_jerk_rad_s3 > 0.0f
        && std::isfinite(config.tracking_gain_per_s)
        && config.tracking_gain_per_s > 0.0f
        && std::isfinite(config.damping_gain_per_s)
        && config.damping_gain_per_s > 0.0f
        && std::isfinite(config.velocity_epsilon)
        && config.velocity_epsilon >= 0.0f
        && std::isfinite(config.minimum_dt_s)
        && config.minimum_dt_s > 0.0f
        && std::isfinite(config.maximum_dt_s)
        && config.maximum_dt_s >= config.minimum_dt_s;
}

} // namespace

bool VelocityPlanner::init(const VelocityPlannerConfig& config)
{
    if (!configValid(config)) {
        initialized_ = false;
        reset();
        return false;
    }

    config_ = config;
    initialized_ = true;
    reset();
    return true;
}

void VelocityPlanner::reset()
{
    reset(0.0f, 0.0f, 0.0f);
}

void VelocityPlanner::reset(float vx, float vy, float wz)
{
    plan_.vx = std::isfinite(vx) ? vx : 0.0f;
    plan_.vy = std::isfinite(vy) ? vy : 0.0f;
    plan_.wz = std::isfinite(wz) ? wz : 0.0f;
    plan_.ax = 0.0f;
    plan_.ay = 0.0f;
    plan_.alpha = 0.0f;
    acceleration_x_ = 0.0f;
    acceleration_y_ = 0.0f;
    angular_acceleration_ = 0.0f;
}

VelocityPlan VelocityPlanner::update(
    float target_vx,
    float target_vy,
    float target_wz,
    float dt_s
)
{
    if (!initialized_) {
        reset();
        return plan_;
    }

    const float dt = clampValue(
        std::isfinite(dt_s) ? dt_s : config_.minimum_dt_s,
        config_.minimum_dt_s,
        config_.maximum_dt_s
    );
    target_vx = std::isfinite(target_vx) ? target_vx : 0.0f;
    target_vy = std::isfinite(target_vy) ? target_vy : 0.0f;
    target_wz = std::isfinite(target_wz) ? target_wz : 0.0f;

    const float error_x = target_vx - plan_.vx;
    const float error_y = target_vy - plan_.vy;
    float desired_ax =
        config_.tracking_gain_per_s * error_x
        - config_.damping_gain_per_s * acceleration_x_;
    float desired_ay =
        config_.tracking_gain_per_s * error_y
        - config_.damping_gain_per_s * acceleration_y_;

    // 先限制期望加速度的二维合成模长，再限制加速度向量的变化率。

    //判断加速度和速度方向是否一致
    const float linear_limit =
        desired_ax * plan_.vx + desired_ay * plan_.vy >= 0.0f
        ? config_.linear_accel_mps2
        : config_.linear_decel_mps2;

    limitVector(&desired_ax, &desired_ay, linear_limit);
    const float previous_acceleration_x = acceleration_x_;
    const float previous_acceleration_y = acceleration_y_;
    float delta_ax = desired_ax - acceleration_x_;
    float delta_ay = desired_ay - acceleration_y_;
    limitVector(
        &delta_ax,
        &delta_ay,
        config_.linear_jerk_mps3 * dt
    );
    acceleration_x_ += delta_ax;
    acceleration_y_ += delta_ay;

    plan_.vx += acceleration_x_ * dt;
    plan_.vy += acceleration_y_ * dt;

    const float acceleration_capture =
        config_.linear_jerk_mps3 * dt;
    if (std::fabs(target_vx - plan_.vx)
            <= config_.velocity_epsilon
        && std::fabs(previous_acceleration_x) <= acceleration_capture
        && std::fabs(acceleration_x_) <= acceleration_capture) {
        plan_.vx = target_vx;
        acceleration_x_ = 0.0f;
    }
    if (std::fabs(target_vy - plan_.vy)
            <= config_.velocity_epsilon
        && std::fabs(previous_acceleration_y) <= acceleration_capture
        && std::fabs(acceleration_y_) <= acceleration_capture) {
        plan_.vy = target_vy;
        acceleration_y_ = 0.0f;
    }

    const float yaw_error = target_wz - plan_.wz;
    const float previous_angular_acceleration = angular_acceleration_;
    const float desired_alpha =
        config_.tracking_gain_per_s * yaw_error
        - config_.damping_gain_per_s * angular_acceleration_;
    const float yaw_limit =
        desired_alpha * plan_.wz >= 0.0f
        ? config_.yaw_accel_rad_s2
        : config_.yaw_decel_rad_s2;
    const float limited_alpha = clampValue(
        desired_alpha,
        -yaw_limit,
        yaw_limit
    );
    angular_acceleration_ = moveToward(
        angular_acceleration_,
        limited_alpha,
        config_.yaw_jerk_rad_s3 * dt
    );

    plan_.wz += angular_acceleration_ * dt;
    const float angular_capture =
        config_.yaw_jerk_rad_s3 * dt;
    if (std::fabs(target_wz - plan_.wz)
            <= config_.velocity_epsilon
        && std::fabs(previous_angular_acceleration) <= angular_capture
        && std::fabs(angular_acceleration_) <= angular_capture) {
        plan_.wz = target_wz;
        angular_acceleration_ = 0.0f;
    }

    plan_.ax = acceleration_x_;
    plan_.ay = acceleration_y_;
    plan_.alpha = angular_acceleration_;
    return plan_;
}

} // namespace chassis_app
