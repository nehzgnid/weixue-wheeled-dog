#include "usb_cmd_handler.h"
#include "st_servo.h"
#include <string.h>

// 接收缓冲区
uint8_t usb_cmd_buffer[PACKET_SIZE];

// 默认运动参数
#define CMD_DEFAULT_SPEED 0   // 0代表最快速度
#define CMD_DEFAULT_ACC   0   // 0代表最大加速度

// ---------------------------------------------------------
// USB 接收回调 (在中断上下文运行，需极速处理)
// ---------------------------------------------------------
void USB_Data_Rx_Handler(uint8_t* Buf, uint32_t Len) {
    // 1. 长度校验
    if (Len != PACKET_SIZE) return;
    
    // 2. 帧头校验
    if (Buf[0] != CMD_HEAD_1 || Buf[1] != CMD_HEAD_2) return;
    
    // 3. 简单校验和检查 (可选，为了速度也可跳过，这里演示加上)
    uint8_t sum = 0;
    for (uint32_t i = 0; i < Len - 1; i++) {
        sum += Buf[i];
    }
    if (sum != Buf[Len - 1]) return; // 校验失败
    
    // 4. 拷贝数据到全局缓冲区
    memcpy(usb_cmd_buffer, Buf, PACKET_SIZE);
    
    // 5. 唤醒舵机任务 (释放信号量)
    // 这里的 ServoCmdSemHandle 需要在 freertos.c 中定义并初始化
    if (ServoCmdSemHandle != NULL) {
        osSemaphoreRelease(ServoCmdSemHandle);
    }
}

// ---------------------------------------------------------
// 舵机处理循环 (在任务中运行)
// ---------------------------------------------------------
void Servo_Process_Loop(void) {
    // 准备发送给舵机的数组
    uint8_t ids[SERVO_COUNT];
    int16_t positions[SERVO_COUNT];
    uint16_t speeds[SERVO_COUNT];
    uint8_t accs[SERVO_COUNT];
    
    // 指针指向数据区 (跳过头2字节)
    uint8_t *pData = &usb_cmd_buffer[2];
    
    for(int i = 0; i < SERVO_COUNT; i++) {
        // 解析格式: [ID, PosL, PosH, SpdL, SpdH, Acc]
        ids[i] = *pData++;
        
        uint8_t pl = *pData++;
        uint8_t ph = *pData++;
        positions[i] = (int16_t)(pl | (ph << 8));
        
        uint8_t sl = *pData++;
        uint8_t sh = *pData++;
        speeds[i] = (uint16_t)(sl | (sh << 8));
        
        accs[i] = *pData++;
    }
    
    // 调用总线驱动，立即发送
    ST_SyncWritePos(ids, SERVO_COUNT, positions, speeds, accs);
}
