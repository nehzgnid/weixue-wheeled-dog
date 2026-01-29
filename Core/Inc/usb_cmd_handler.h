#ifndef __USB_CMD_HANDLER_H
#define __USB_CMD_HANDLER_H

#include "main.h"
#include "cmsis_os.h"

// === 1. 协议基础 ===
#define PROTOCOL_HEAD_1 0xA5
#define PROTOCOL_HEAD_2 0x5A

// --- 包类型定义 (Packet Types) ---
#define TYPE_PING            0x01  // 心跳/延迟测试
#define TYPE_SERVO_CTRL      0x10  // 舵机控制指令 (Host -> MCU)
#define TYPE_SERVO_FB        0x20  // 舵机状态反馈 (MCU -> Host)
#define TYPE_SENSOR_IMU      0x30  // IMU 传感器数据 (预留)
#define TYPE_SYS_ERROR       0xE0  // 错误告警

// === 2. 规模配置 ===
#define MAX_SERVO_COUNT      18    // 系统支持的最大舵机数
#define COMM_PAYLOAD_MAX     120   // 通信任务单包最大载荷 (总包128)

// === 3. 数据结构 (1字节对齐) ===
#pragma pack(1)

// 舵机单机控制参数 (6 bytes)
typedef struct {
    uint8_t id;
    int16_t pos;
    int16_t speed;
    uint8_t acc;
} ServoCtrlParam_t;

// 舵机单机反馈参数 (7 bytes)
typedef struct {
    uint8_t id;
    int16_t pos;
    int16_t speed;
    int16_t load;
} ServoFBParam_t;

// 通用通信包载荷 (由 TxTask 统一发送)
typedef struct {
    uint8_t type;
    uint8_t len;
    uint8_t data[COMM_PAYLOAD_MAX];
} CommPacket_t;

#pragma pack()

// === 4. 接口声明 ===
extern osMessageQueueId_t CommTxQueueHandle;
extern osMessageQueueId_t ServoCmdQueueHandle;
extern osMutexId_t ServoUartMutexHandle;

void USB_RingBuffer_Push(uint8_t* buf, uint32_t len);
uint32_t USB_RingBuffer_Pop(uint8_t* byte);

#endif