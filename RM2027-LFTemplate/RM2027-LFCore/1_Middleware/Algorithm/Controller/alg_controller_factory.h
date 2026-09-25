/**
 * @file alg_controller_factory.h
 * @author WHU-Robomaster Team
 * @brief 控制器工厂类（使用小驼峰命名）
 * @version 1.0
 * @date 2026/09/24
 *
 * @copyright WHU-LuojiaFox (c) 2027
 *
 */

#ifndef ALG_CONTROLLER_FACTORY_H
    #define ALG_CONTROLLER_FACTORY_H

/* Includes ------------------------------------------------------------------*/

#include "alg_controller.h"
#include "alg_pid.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 控制器工厂类
 *        使用工厂模式创建和管理各种控制器实例
 *
 * @note 支持的控制器类型：
 *       - ControllerPid: PID控制器（已实现）
 *       - 未来可扩展：模糊控制、自适应控制、MPC等
 *
 */
class ClassControllerFactory {
public:
    /**
     * @brief 创建控制器实例
     *
     * @param type 控制器类型
     * @return ClassController* 控制器指针（动态分配，调用者负责释放）
     *
     * @warning 返回的指针使用后需要 delete 释放内存
     *
     * @code
     * // 使用示例
     * ClassControllerFactory factory;
     * ClassController* controller = factory.createController(ControllerPid);
     * if (controller) {
     *     controller->init();
     *     // ... 使用控制器 ...
     *     delete controller;  // 释放内存
     * }
     * @endcode
     */
    ClassController* createController(EnumControllerType type);

    /**
     * @brief 创建PID控制器并配置参数
     *
     * @param kp P值
     * @param ki I值
     * @param kd D值
     * @param kf 前馈 (默认: 0.0f)
     * @param iOutMax 积分限幅 (默认: 0.0f)
     * @param outMax 输出限幅 (默认: 0.0f)
     * @param dt 时间片长度 (默认: 0.001f)
     * @param deadZone 死区误差阈值 (默认: 0.0f)
     * @param iVariableSpeedA 变速积分误差阈值A (默认: 0.0f)
     * @param iVariableSpeedB 变速积分误差阈值B (默认: 0.0f)
     * @param iSeparateThreshold 积分分离误差阈值 (默认: 0.0f)
     * @param dFirst 是否开启微分先行 (默认: PidDFirstDisable)
     * @return ClassController* 配置好的PID控制器指针
     */
    ClassController* createPid(
        float kp,
        float ki,
        float kd,
        float kf = 0.0f,
        float iOutMax = 0.0f,
        float outMax = 0.0f,
        float dt = 0.001f,
        float deadZone = 0.0f,
        float iVariableSpeedA = 0.0f,
        float iVariableSpeedB = 0.0f,
        float iSeparateThreshold = 0.0f,
        EnumPidDFirst dFirst = PidDFirstDisable
    );

private:
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) WHU-LUOJIAFOX **************************/