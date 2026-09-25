/**
 * @file alg_controller.h
 * @author WHU-Robomaster LuojiaFox
 * @brief 控制器抽象基类与工厂模式
 * @version 1.0
 * @date 2026/09/24
 *
 * @copyright WHU-LuojiaFox (c) 2027
 *
 */

#ifndef ALG_CONTROLLER_H
    #define ALG_CONTROLLER_H

/* Includes ------------------------------------------------------------------*/

#include "alg_basic.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 控制器类型枚举
 *
 */
enum EnumControllerType {
    ControllerPid = 0,      // PID控制器
    // TODO: 在此处添加新控制器类型
    // ControllerFuzzy,     // 模糊控制器 (示例)
    // ControllerAdaptive,  // 自适应控制器 (示例)
    // ControllerMpc,       // 模型预测控制器 (示例)
    ControllerTypeCount   // 控制器类型总数
};

/**
 * @brief 控制器抽象基类
 *        定义所有控制器的统一接口（使用小驼峰命名）
 *
 */
class ClassController {
public:
    virtual ~ClassController() = default;

    /**
     * @brief 初始化控制器（纯虚函数，由子类实现）
     *
     */
    virtual void init() = 0;

    /**
     * @brief 计算控制器输出（纯虚函数，由子类实现）
     *        在定时器回调中周期性调用
     *
     */
    virtual void timCalculatePeriodElapsedCallback() = 0;

    /**
     * @brief 获取控制器输出值
     *
     * @return float 输出值
     */
    virtual float getOut() = 0;

    /**
     * @brief 设置目标值
     *
     * @param target 目标值
     */
    virtual void setTarget(float target) = 0;

    /**
     * @brief 设置当前值（反馈值）
     *
     * @param now 当前值
     */
    virtual void setNow(float now) = 0;

    /**
     * @brief 获取控制器类型
     *
     * @return EnumControllerType 控制器类型
     */
    EnumControllerType getControllerType();

protected:
    // 控制器类型
    EnumControllerType controllerType;
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

inline EnumControllerType ClassController::getControllerType() {
    return (controllerType);
}

#endif

/************************ COPYRIGHT(C) WHU-LUOJIAFOX **************************/