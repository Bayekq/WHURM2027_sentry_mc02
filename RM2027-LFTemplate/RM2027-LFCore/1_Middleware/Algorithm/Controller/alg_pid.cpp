/**
 * @file alg_pid.cpp
 * @author yssickjgd 1345578933@qq.com
 * @brief PID算法实现
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "alg_pid.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化控制器（重写基类纯虚函数）
 *        使用默认参数初始化PID控制器
 *
 */
void ClassPID::init() {
    // 设置控制器类型（基类成员）
    controllerType = ControllerPid;

    // 使用默认参数初始化PID
    KP = 0.0f;
    KI = 0.0f;
    KD = 0.0f;
    KF = 0.0f;
    IOutMax = 0.0f;
    OutMax = 0.0f;
    Dt = 0.001f;
    DeadZone = 0.0f;
    IVariableSpeedA = 0.0f;
    IVariableSpeedB = 0.0f;
    ISeparateThreshold = 0.0f;
    DFirst = PidDFirstDisable;

    // 清零内部状态
    PreNow = 0.0f;
    PreTarget = 0.0f;
    PreOut = 0.0f;
    PreError = 0.0f;
    Out = 0.0f;
    Target = 0.0f;
    Now = 0.0f;
    IntegralError = 0.0f;
}

/**
 * @brief PID参数配置（PID特有方法）
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
 */
void ClassPID::init(
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
    // 设置控制器类型（基类成员）
    controllerType = ControllerPid;

    // 配置PID参数
    KP = kp;
    KI = ki;
    KD = kd;
    KF = kf;
    IOutMax = iOutMax;
    OutMax = outMax;
    Dt = dt;
    DeadZone = deadZone;
    IVariableSpeedA = iVariableSpeedA;
    IVariableSpeedB = iVariableSpeedB;
    ISeparateThreshold = iSeparateThreshold;
    DFirst = dFirst;
}

/**
 * @brief PID调整值, 计算周期与D_T相同
 *
 * @return float 输出值
 */
void ClassPID::timCalculatePeriodElapsedCallback() {
    // P输出
    float pOut = 0.0f;
    // I输出
    float iOut = 0.0f;
    // D输出
    float dOut = 0.0f;
    // F输出
    float fOut = 0.0f;
    // 误差
    float error = 0.0f;
    // 绝对值误差
    float absError = 0.0f;
    // 线性变速积分
    float speedRatio = 0.0f;

    error = Target - Now;
    absError = Math_Abs(error);

    // 判断死区
    if (absError < DeadZone) {
        Target = Now;
        error = 0.0f;
        absError = 0.0f;
    } else if (error > 0.0f && absError > DeadZone) {
        error -= DeadZone;
    } else if (error < 0.0f && absError > DeadZone) {
        error += DeadZone;
    }

    // 计算p项

    pOut = KP * error;

    // 计算i项

    if (IVariableSpeedA == 0.0f && IVariableSpeedB == 0.0f) {
        // 非变速积分
        speedRatio = 1.0f;
    } else {
        // 变速积分
        if (absError <= IVariableSpeedA) {
            speedRatio = 1.0f;
        } else if (IVariableSpeedA < absError && absError < IVariableSpeedB) {
            speedRatio =
                (IVariableSpeedB - absError) / (IVariableSpeedB - IVariableSpeedA);
        } else if (absError >= IVariableSpeedB) {
            speedRatio = 0.0f;
        }
    }
    // 积分限幅
    if (IOutMax != 0.0f) {
        BasicMath::constrain(&IntegralError, -IOutMax / KI, IOutMax / KI);
    }
    if (ISeparateThreshold == 0.0f) {
        // 没有积分分离
        IntegralError += speedRatio * Dt * error;
        iOut = KI * IntegralError;
    } else {
        // 有积分分离
        if (absError < ISeparateThreshold) {
            // 不在积分分离区间上
            IntegralError += speedRatio * Dt * error;
            iOut = KI * IntegralError;
        } else {
            // 在积分分离区间上
            IntegralError = 0.0f;
            iOut = 0.0f;
        }
    }

    // 计算d项

    if (DFirst == PidDFirstDisable) {
        // 没有微分先行
        dOut = KD * (error - PreError) / Dt;
    } else {
        // 微分先行使能
        dOut = -KD * (Now - PreNow) / Dt;
    }

    // 计算前馈

    fOut = KF * (Target - PreTarget);

    // 计算输出

    Out = pOut + iOut + dOut + fOut;

    // 输出限幅
    if (OutMax != 0.0f) {
        BasicMath::constrain(&Out, -OutMax, OutMax);
    }

    // 善后工作
    PreNow = Now;
    PreTarget = Target;
    PreOut = Out;
    PreError = error;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/