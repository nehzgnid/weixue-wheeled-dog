#ifndef __IMU_HANDLER_H
#define __IMU_HANDLER_H

#include "main.h"

// JY901S 协议常量
#define IMU_HEAD_1      0x55
#define IMU_TYPE_TIME   0x50
#define IMU_TYPE_ACC    0x51
#define IMU_TYPE_GYRO   0x52
#define IMU_TYPE_ANGLE  0x53

// 缓冲区大小 (DMA Circular Buffer)
#define IMU_DMA_BUF_SIZE 128

// IMU 数据结构体 (单位: g, deg/s, deg)
typedef struct {
    float acc[3];   // X, Y, Z
    float gyro[3];  // X, Y, Z
    float angle[3]; // Roll, Pitch, Yaw
    uint8_t updated; // 标志位
} IMU_Data_t;

// 全局变量声明
extern IMU_Data_t g_imu_data;

// 函数声明
void IMU_Init(void);
void IMU_Parse_Loop(void);

#endif
