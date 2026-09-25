/**
 * @file alg_controller_factory.cpp
 * @author WHU-Robomaster Team
 * @brief 控制器工厂实现
 * @version 1.0
 * @date 2026/09/24
 *
 * @copyright WHU-LuojiaFox (c) 2027
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_controller_factory.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 创建控制器实例
 *
 * @param type 控制器类型
 * @return ClassController* 控制器指针
 *
 * @note 工厂方法：根据类型创建对应的控制器对象
 *       新增控制器时需要在此处添加case分支
 *
 */
ClassController* ClassControllerFactory::createController(EnumControllerType type) {
    switch (type) {
        case ControllerPid: {
            ClassPID* pid = new ClassPID();
            pid->init();  // 调用无参数初始化（重写基类纯虚函数）
            return pid;
        }

        // TODO: 在此处添加新控制器的创建逻辑
        /*
        case ControllerFuzzy: {
            ClassFuzzy* fuzzy = new ClassFuzzy();
            fuzzy->controllerType = ControllerFuzzy;
            return fuzzy;
        }
        */

        default:
            return nullptr;
    }
}

/**
 * @brief 创建并配置PID控制器
 *
 * @param kp P值
 * @param ki I值
 * @param kd D值
 * @param kf 前馈
 * @param iOutMax 积分限幅
 * @param outMax 输出限幅
 * @param dt 时间片长度
 * @param deadZone 死区误差阈值
 * @param iVariableSpeedA 变速积分误差阈值A
 * @param iVariableSpeedB 变速积分误差阈值B
 * @param iSeparateThreshold 积分分离误差阈值
 * @param dFirst 是否开启微分先行
 * @return ClassController* 配置好的PID控制器指针
 *
 * @note 便捷方法：一步完成PID控制器的创建和配置
 *
 */
ClassController* ClassControllerFactory::createPid(
    float kp,
    float ki,
    float kd,
    float kf,
    float iOutMax,
    float outMax,
    float dt,
    float deadZone,
    float iVariableSpeedA,
    float iVariableSpeedB,
    float iSeparateThreshold,
    EnumPidDFirst dFirst
) {
    ClassPID* pid = new ClassPID();
    pid->init(kp, ki, kd, kf, iOutMax, outMax, dt,
              deadZone, iVariableSpeedA, iVariableSpeedB,
              iSeparateThreshold, dFirst);
    return pid;
}

/************************ COPYRIGHT(C) WHU-LUOJIAFOX **************************/