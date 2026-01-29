#ifndef __USB_CMD_HANDLER_H
#define __USB_CMD_HANDLER_H

#include "main.h"
#include "cmsis_os.h"

// === 协议定义 ===
// 帧头: 0xA5 0x5A
#define CMD_HEAD_1 0xA5
#define CMD_HEAD_2 0x5A

// 控制舵机数量
#define SERVO_COUNT 6

// 每个舵机数据长度: ID(1) + POS_L(1) + POS_H(1) + SPD_L(1) + SPD_H(1) + ACC(1) = 6字节
#define BYTES_PER_SERVO 6

// 包总长: Head(2) + Data(6*6) + Checksum(1) = 39字节
#define PACKET_SIZE (2 + (SERVO_COUNT * BYTES_PER_SERVO) + 1)

// === 全局变量 ===
// 用于 USB 中断和任务间共享数据
extern uint8_t usb_cmd_buffer[PACKET_SIZE];
extern osSemaphoreId_t ServoCmdSemHandle; // 信号量句柄

// === 函数声明 ===
// 供 usbd_cdc_if.c 调用
void USB_Data_Rx_Handler(uint8_t* Buf, uint32_t Len);

// 供 freertos.c 任务调用
void Servo_Process_Loop(void);

#endif
