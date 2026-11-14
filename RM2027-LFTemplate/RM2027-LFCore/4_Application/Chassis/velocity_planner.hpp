/**
 * @file velocity_planner.hpp
 * @brief 底盘速度、加速度和加加速度规划器。
 */

#ifndef VELOCITY_PLANNER_HPP
#define VELOCITY_PLANNER_HPP

#include <stdint.h>

namespace chassis_app {

/** @brief 速度规划参数。 */
struct VelocityPlannerConfig {
    /** @brief 线速度加速上限，m/s²。 */
    float linear_accel_mps2;
    /** @brief 线速度减速上限，m/s²；可以高于加速上限。 */
    float linear_decel_mps2;
    /** @brief 线加速度合成向量的变化率上限，m/s³。 */
    float linear_jerk_mps3;
    /** @brief Yaw 角加速度上限，rad/s²。 */
    float yaw_accel_rad_s2;
    /** @brief Yaw 角减速度上限，rad/s²。 */
    float yaw_decel_rad_s2;
    /** @brief Yaw 角加加速度上限，rad/s³。 */
    float yaw_jerk_rad_s3;
    /** @brief 速度误差到期望加速度的比例增益，1/s。 */
    float tracking_gain_per_s;
    /** @brief 规划加速度到期望值的阻尼增益，1/s。 */
    float damping_gain_per_s;
    /** @brief 速度死区，m/s 或 rad/s。 */
    float velocity_epsilon;
    /** @brief 合法控制周期下限，s。 */
    float minimum_dt_s;
    /** @brief 合法控制周期上限，s。 */
    float maximum_dt_s;
};

/** @brief 本周期规划结果。 */
struct VelocityPlan {
    float vx;
    float vy;
    float wz;
    float ax;
    float ay;
    float alpha;
};

/**
 * @brief 将阶跃速度目标转换为满足加速度与 Jerk 限制的连续目标。
 * @note 规划器只处理正常运动目标，不承担故障急停。安全状态机禁止输出时，
 *       底盘必须直接停车并调用 reset()。
 */
class VelocityPlanner {
public:
    /**
     * @brief 初始化规划器。
     * @param config 物理限制和跟踪参数。
     * @return 参数有效时返回 true。
     */
    bool init(const VelocityPlannerConfig& config);

    /** @brief 将规划速度、加速度和内部状态清零。 */
    void reset();

    /**
     * @brief 从当前实测速度重新开始规划。
     * @note 适用于底盘重新使能，避免规划器从错误的零速度状态产生突变。
     */
    void reset(float vx, float vy, float wz);

    /**
     * @brief 执行一次规划。
     * @param target_vx 目标 x 向速度，m/s。
     * @param target_vy 目标 y 向速度，m/s。
     * @param target_wz 目标旋转角速度，rad/s。
     * @param dt_s 控制周期，s。
     * @return 平滑后的速度目标和对应加速度。
     */
    VelocityPlan update(
        float target_vx,
        float target_vy,
        float target_wz,
        float dt_s
    );

    /** @brief 获取最近一次规划结果。 */
    const VelocityPlan& plan() const
    {
        return plan_;
    }

    /** @brief 查询规划器是否已成功初始化。 */
    bool initialized() const
    {
        return initialized_;
    }

private:
    VelocityPlannerConfig config_ {};
    VelocityPlan plan_ {};
    float acceleration_x_ = 0.0f;
    float acceleration_y_ = 0.0f;
    float angular_acceleration_ = 0.0f;
    bool initialized_ = false;
};

} // namespace chassis_app

#endif // VELOCITY_PLANNER_HPP
