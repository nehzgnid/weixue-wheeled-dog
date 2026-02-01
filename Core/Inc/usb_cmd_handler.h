#ifndef __USB_CMD_HANDLER_H
#define __USB_CMD_HANDLER_H

#include "main.h"
#include "cmsis_os.h"

// === 1. 协议基础 ===
#define PROTOCOL_HEAD_1 0xA5
#define PROTOCOL_HEAD_2 0x5A

// --- 包类型定义 (Packet Types) ---
#define TYPE_PING            0x01  
#define TYPE_SERVO_CTRL      0x10  
#define TYPE_TORQUE_CTRL     0x11  
#define TYPE_SERVO_FB        0x20  
#define TYPE_SENSOR_IMU      0x30  
#define TYPE_RL_STATE        0x40  // 新增: 强化学习专用全状态包
#define TYPE_SYS_ERROR       0xE0  

// === 2. 规模配置 ===
#define MAX_SERVO_COUNT      18    
#define COMM_PAYLOAD_MAX     128   // 稍微加大一点以容纳 RL 包

// === 3. 数据结构 (1字节对齐) ===
#pragma pack(1)

// 舵机单机控制参数 (6 bytes)
typedef struct __attribute__((packed)) { // 加上这个！防止编译器插入 Padding
    uint8_t id;
    int16_t pos;
    uint16_t speed;
    uint8_t acc;
} ServoCtrlParam_t;

// 舵机单机反馈参数 (7 bytes)
typedef struct {
    uint8_t id;
    int16_t pos;
    int16_t speed;
    int16_t load;
} ServoFBParam_t;

// 通用通信包载荷
typedef struct {
    uint8_t type;
    uint8_t len;
    uint8_t data[COMM_PAYLOAD_MAX];
} CommPacket_t;

#pragma pack()

// === 4. 接口声明 ===
extern osMessageQueueId_t CommTxQueueHandle;
extern osMessageQueueId_t ServoCmdQueueHandle;
// extern osMutexId_t ServoUartMutexHandle; // 废弃，改用 V1/V2

void USB_RingBuffer_Push(uint8_t* buf, uint32_t len);
uint32_t USB_RingBuffer_Pop(uint8_t* byte);

#endif
