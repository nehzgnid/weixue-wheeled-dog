#ifndef __ST_SERVO_H
#define __ST_SERVO_H

#include "main.h"

// 移除硬编码的宏定义，改为函数传参
// extern UART_HandleTypeDef huart2;
// #define SERVO_UART &huart2

// ST系列舵机寄存器地址
#define STS_ID                  5
#define STS_BAUD_RATE           6
#define STS_MIN_ANGLE_LIMIT_L   9
#define STS_MAX_ANGLE_LIMIT_L   11
#define STS_TORQUE_ENABLE       40
#define STS_ACC                 41
#define STS_GOAL_POSITION_L     42
#define STS_GOAL_TIME_L         44
#define STS_GOAL_SPEED_L        46
#define STS_LOCK                55
#define STS_PRESENT_POSITION_L  56

// 指令
#define INST_PING       0x01
#define INST_READ       0x02
#define INST_WRITE      0x03
#define INST_REG_WRITE  0x04
#define INST_ACTION     0x05
#define INST_SYNC_WRITE 0x83

// API 函数声明 (增加 huart 参数)
void ST_WritePos(UART_HandleTypeDef *huart, uint8_t id, int16_t pos, uint16_t speed, uint8_t acc);
void ST_SetTorque(UART_HandleTypeDef *huart, uint8_t id, uint8_t enable);
void ST_SyncWritePos(UART_HandleTypeDef *huart, uint8_t *ids, uint8_t num, int16_t *pos, uint16_t *speed, uint8_t *acc);
int8_t ST_ReadInfo(UART_HandleTypeDef *huart, uint8_t id, int16_t *pos, int16_t *speed, int16_t *load);

#endif