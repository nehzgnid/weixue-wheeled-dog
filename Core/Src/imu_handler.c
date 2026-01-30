#include "imu_handler.h"
#include "usart.h"
#include <string.h>

// 引用 USART3 句柄 (由 CubeMX 生成)
extern UART_HandleTypeDef huart3;

// DMA 接收缓冲区
uint8_t imu_dma_buf[IMU_DMA_BUF_SIZE];
// 全局数据实例
IMU_Data_t g_imu_data;

// 初始化：启动 DMA 接收
void IMU_Init(void) {
    HAL_UART_Receive_DMA(&huart3, imu_dma_buf, IMU_DMA_BUF_SIZE);
}

// 内部函数：解析单包数据 (11字节)
static void Parse_Packet(uint8_t *packet) {
    if (packet[0] != 0x55) return;
    
    // 校验和 (Sum of 0..9)
    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) sum += packet[i];
    if (sum != packet[10]) return; // 校验失败
    
    int16_t temp[3]; // 原始数据
    
    switch (packet[1]) {
        case IMU_TYPE_ACC:
            temp[0] = (short)(packet[3]<<8 | packet[2]);
            temp[1] = (short)(packet[5]<<8 | packet[4]);
            temp[2] = (short)(packet[7]<<8 | packet[6]);
            g_imu_data.acc[0] = (float)temp[0] / 32768.0f * 16.0f; // ±16g
            g_imu_data.acc[1] = (float)temp[1] / 32768.0f * 16.0f;
            g_imu_data.acc[2] = (float)temp[2] / 32768.0f * 16.0f;
            break;
            
        case IMU_TYPE_GYRO:
            temp[0] = (short)(packet[3]<<8 | packet[2]);
            temp[1] = (short)(packet[5]<<8 | packet[4]);
            temp[2] = (short)(packet[7]<<8 | packet[6]);
            g_imu_data.gyro[0] = (float)temp[0] / 32768.0f * 2000.0f; // ±2000deg/s
            g_imu_data.gyro[1] = (float)temp[1] / 32768.0f * 2000.0f;
            g_imu_data.gyro[2] = (float)temp[2] / 32768.0f * 2000.0f;
            break;
            
        case IMU_TYPE_ANGLE:
            temp[0] = (short)(packet[3]<<8 | packet[2]);
            temp[1] = (short)(packet[5]<<8 | packet[4]);
            temp[2] = (short)(packet[7]<<8 | packet[6]);
            g_imu_data.angle[0] = (float)temp[0] / 32768.0f * 180.0f; // Roll
            g_imu_data.angle[1] = (float)temp[1] / 32768.0f * 180.0f; // Pitch
            g_imu_data.angle[2] = (float)temp[2] / 32768.0f * 180.0f; // Yaw
            g_imu_data.updated = 1;
            break;
    }
}

// 循环解析 (需在任务中周期调用)
// 维护一个读指针，追赶 DMA 的写指针 (CNDTR)
static uint32_t read_ptr = 0;

void IMU_Parse_Loop(void) {
    // 获取 DMA 当前写入位置
    // CNDTR 是倒计数的，所以写入位置 = Total - CNDTR
    uint32_t write_ptr = IMU_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
    
    while (read_ptr != write_ptr) {
        // 计算当前有效数据长度
        uint32_t available = 0;
        if (write_ptr >= read_ptr) available = write_ptr - read_ptr;
        else available = IMU_DMA_BUF_SIZE - read_ptr + write_ptr;
        
        // 至少要有 11 字节才能解析一包
        if (available < 11) break;
        
        // 检查帧头 0x55
        if (imu_dma_buf[read_ptr] == 0x55) {
            // 尝试读取完整一包
            uint8_t packet[11];
            for(int i=0; i<11; i++) {
                packet[i] = imu_dma_buf[(read_ptr + i) % IMU_DMA_BUF_SIZE];
            }
            
            // 解析
            Parse_Packet(packet);
            
            // 成功解析一包，步进11
            read_ptr = (read_ptr + 11) % IMU_DMA_BUF_SIZE;
        } else {
            // 帧头不对，步进1
            read_ptr = (read_ptr + 1) % IMU_DMA_BUF_SIZE;
        }
    }
}
