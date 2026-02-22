#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// 电机数量
#define MOTOR_COUNT 4

// PID 参数结构体
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float Kf;           // 前馈系数
    
    float Error;        // 当前误差
    float LastError;    // 上一次误差
    float Integral;     // 积分值
    
    float IntegralLimit; // 积分限幅
    float OutputLimit;   // 输出限幅
} PID_TypeDef;

// 电机控制模式
typedef enum {
    MOTOR_MODE_PWM = 0, // 开环 PWM 直接控制
    MOTOR_MODE_SPEED = 1 // 闭环 PID 速度控制
} Motor_ControlMode_TypeDef;

// 电机控制结构体
typedef struct {
    float TargetSpeed;   // 目标速度
    float CurrentSpeed;  // 当前速度 (来自反馈)
    int16_t PWM_Output;  // 计算出的PWM输出值 (-3600 ~ 3600)
    PID_TypeDef PID;     // PID控制器
} Motor_TypeDef;

// 供外部访问的电机数组
extern volatile Motor_TypeDef Motors[MOTOR_COUNT];
extern volatile Motor_ControlMode_TypeDef GlobalControlMode;
extern volatile uint32_t Motor_RxCount;
extern volatile uint32_t Motor_RxIdleCount;
extern volatile uint32_t Motor_RxErrorCount; // 新增
extern volatile uint8_t Motor_Initialized;   // 新增：初始化标志位
extern char Motor_LastRawPacket[64];

// 函数声明
void Motor_Init(void);
void Motor_SetControlMode(Motor_ControlMode_TypeDef mode); // 设置控制模式
void Motor_SetSpeed(uint8_t motor_index, float speed);
void Motor_SetAllSpeed(float speed1, float speed2, float speed3, float speed4);
void Motor_SendCmd(const char* cmd); // 暴露底层发送函数
void Motor_Send_PWM(void); // 发送 $pwm 到驱动板
void Motor_Control_Loop(void); // 放在定时器中调用，进行PID计算和发送
void Process_RingBuffer(void); // 暴露给外部调用
void Motor_UART5_RxHandler(void); // 供 UART5_IRQHandler 调用

// 用于调试或手动发送指令
void Motor_SendCmd(const char* cmd);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
