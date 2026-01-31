#include "st_servo.h"
#include <string.h>

// 内部计算校验和函数
static uint8_t CheckSum(uint8_t *buf, uint8_t len) {
    uint8_t sum = 0;
    int i;
    for (i = 2; i < len - 1; i++) {
        sum += buf[i];
    }
    return ~sum;
}

/**
 * @brief  单个舵机位置控制
 */
void ST_WritePos(UART_HandleTypeDef *huart, uint8_t id, int16_t pos, uint16_t speed, uint8_t acc) {
    uint8_t tx_buf[14];
    tx_buf[0] = 0xFF;
    tx_buf[1] = 0xFF;
    tx_buf[2] = id;
    tx_buf[3] = 9; 
    tx_buf[4] = INST_WRITE;
    tx_buf[5] = STS_ACC;
    tx_buf[6] = acc;
    tx_buf[7] = (uint8_t)(pos & 0xFF);
    tx_buf[8] = (uint8_t)((pos >> 8) & 0xFF);
    tx_buf[9] = 0;
    tx_buf[10] = 0;
    tx_buf[11] = (uint8_t)(speed & 0xFF);
    tx_buf[12] = (uint8_t)((speed >> 8) & 0xFF);
    tx_buf[13] = CheckSum(tx_buf, 14);

    HAL_UART_Transmit(huart, tx_buf, 14, 10);
}

/**
 * @brief  设置扭矩开关
 */
void ST_SetTorque(UART_HandleTypeDef *huart, uint8_t id, uint8_t enable) {
    uint8_t buf[8];
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = 4;
    buf[4] = INST_WRITE;
    buf[5] = STS_TORQUE_ENABLE;
    buf[6] = enable ? 1 : 0;
    buf[7] = CheckSum(buf, 8);
    
    HAL_UART_Transmit(huart, buf, 8, 10);
}

/**
 * @brief  同步控制多个舵机
 */
void ST_SyncWritePos(UART_HandleTypeDef *huart, uint8_t *ids, uint8_t num, int16_t *pos, uint16_t *speed, uint8_t *acc) {
    uint8_t data_len_per_servo = 7;
    uint8_t buf[200];
    uint8_t idx = 0;
    int i;
    
    buf[idx++] = 0xFF;
    buf[idx++] = 0xFF;
    buf[idx++] = 0xFE;  
    buf[idx++] = (data_len_per_servo + 1) * num + 4; 
    buf[idx++] = INST_SYNC_WRITE;
    buf[idx++] = STS_ACC; 
    buf[idx++] = data_len_per_servo; 
    
    for (i = 0; i < num; i++) {
        buf[idx++] = ids[i];
        buf[idx++] = acc[i];
        buf[idx++] = (uint8_t)(pos[i] & 0xFF);
        buf[idx++] = (uint8_t)((pos[i] >> 8) & 0xFF);
        buf[idx++] = 0;
        buf[idx++] = 0;
        buf[idx++] = (uint8_t)(speed[i] & 0xFF);
        buf[idx++] = (uint8_t)((speed[i] >> 8) & 0xFF);
    }
    
    buf[idx] = CheckSum(buf, idx + 1);
    idx++;
    
    HAL_UART_Transmit(huart, buf, idx, 50);
}

/**
 * @brief  读取舵机信息
 */
int8_t ST_ReadInfo(UART_HandleTypeDef *huart, uint8_t id, int16_t *pos, int16_t *speed, int16_t *load) {
    uint8_t tx_buf[8];
    uint8_t rx_buf[14]; 
    
    tx_buf[0] = 0xFF;
    tx_buf[1] = 0xFF;
    tx_buf[2] = id;
    tx_buf[3] = 4;
    tx_buf[4] = INST_READ;
    tx_buf[5] = STS_PRESENT_POSITION_L; 
    tx_buf[6] = 6; 
    tx_buf[7] = CheckSum(tx_buf, 8);
    
    if (HAL_UART_Transmit(huart, tx_buf, 8, 2) != HAL_OK) return -1;
    if (HAL_UART_Receive(huart, rx_buf, 12, 5) != HAL_OK) return -1;
    if (rx_buf[0] != 0xFF || rx_buf[1] != 0xFF || rx_buf[2] != id) return -1;
    
    if (pos) *pos = (int16_t)(rx_buf[5] | (rx_buf[6] << 8));
    
    if (speed) {
        uint16_t raw_spd = (uint16_t)(rx_buf[7] | (rx_buf[8] << 8));
        if (raw_spd & 0x8000) *speed = -1 * (int16_t)(raw_spd & 0x7FFF);
        else *speed = (int16_t)raw_spd;
    }
    
    if (load) {
        uint16_t raw_load = (uint16_t)(rx_buf[9] | (rx_buf[10] << 8));
        if (raw_load & 0x8000) *load = -1 * (int16_t)(raw_load & 0x7FFF);
        else if (raw_load & 0x400) *load = -1 * (int16_t)(raw_load & 0x3FF);
        else *load = (int16_t)raw_load;
    }
    
    return 0;
}
