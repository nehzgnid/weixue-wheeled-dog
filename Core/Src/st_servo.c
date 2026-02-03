#include "st_servo.h"
#include <string.h>

// 内部校验和计算
static uint8_t CheckSum(uint8_t *buf, uint8_t len) {
    uint8_t sum = 0;
    int i;
    for(i=2; i<len-1; i++) sum += buf[i];
    return ~sum;
}

void ST_SetTorque(UART_HandleTypeDef *huart, uint8_t id, uint8_t enable) {
    uint8_t tx[9] = {0xFF, 0xFF, id, 0x05, 0x03, 0x28, enable ? 1 : 0, 0, 0};
    tx[8] = CheckSum(tx, 9);
    HAL_UART_Transmit(huart, tx, 9, 2);
}

// 单次读取 (保留作为备份或调试)
int8_t ST_ReadInfo(UART_HandleTypeDef *huart, uint8_t id, int16_t *pos, int16_t *speed, int16_t *load) {
    uint8_t tx_buf[8] = {0xFF, 0xFF, id, 0x04, 0x02, 0x38, 0x06, 0};
    uint8_t rx_buf[12];
    tx_buf[7] = CheckSum(tx_buf, 8);
    
    // Clear ORE
    volatile uint32_t tmp;
    tmp = huart->Instance->SR;
    tmp = huart->Instance->DR;
    (void)tmp;

    if (HAL_UART_Transmit(huart, tx_buf, 8, 2) != HAL_OK) return -1;
    if (HAL_UART_Receive(huart, rx_buf, 12, 2) != HAL_OK) return -1;
    
    if (rx_buf[0] != 0xFF || rx_buf[1] != 0xFF || rx_buf[2] != id) return -1;
    
    if (pos) *pos = (int16_t)(rx_buf[5] | (rx_buf[6] << 8));
    if (speed) *speed = (int16_t)(rx_buf[7] | (rx_buf[8] << 8));
    if (load) *load = (int16_t)(rx_buf[9] | (rx_buf[10] << 8));
    return 0;
}

// 同步写 (控制核心)
void ST_SyncWritePos(UART_HandleTypeDef *huart, uint8_t *ids, uint8_t count, int16_t *pos, uint16_t *speed, uint8_t *acc) {
    if (count > 6) return;
    uint8_t tx_buf[64];
    uint8_t idx = 0;
    
    tx_buf[idx++] = 0xFF; tx_buf[idx++] = 0xFF; tx_buf[idx++] = 0xFE;
    tx_buf[idx++] = count * 6 + 4;
    tx_buf[idx++] = 0x83; // SYNC WRITE
    tx_buf[idx++] = 0x2A; // Addr: Target Pos
    tx_buf[idx++] = 0x05; // Per-Servo Data Len (Pos2+Spd2+Acc1)
    
    uint8_t sum = 0xFE + tx_buf[3] + 0x83 + 0x2A + 0x05;
    
    for(int i=0; i<count; i++) {
        tx_buf[idx++] = ids[i]; sum += ids[i];
        
        uint16_t p = pos[i];
        uint16_t s = speed[i];
        uint8_t a = acc[i];
        
        tx_buf[idx++] = p & 0xFF; sum += p & 0xFF;
        tx_buf[idx++] = (p >> 8) & 0xFF; sum += (p >> 8) & 0xFF;
        tx_buf[idx++] = s & 0xFF; sum += s & 0xFF;
        tx_buf[idx++] = (s >> 8) & 0xFF; sum += (s >> 8) & 0xFF;
        tx_buf[idx++] = a; sum += a;
    }
    
    tx_buf[idx++] = ~sum;
    HAL_UART_Transmit(huart, tx_buf, idx, 5);
}

// 同步读 (性能核心)
int ST_SyncRead(UART_HandleTypeDef *huart, uint8_t *ids, uint8_t count, uint8_t *rx_buf) {
    if (count == 0) return 0;
    
    uint8_t tx_buf[32];
    uint8_t idx = 0;
    uint8_t sum = 0;
    
    tx_buf[idx++] = 0xFF; tx_buf[idx++] = 0xFF; tx_buf[idx++] = 0xFE;
    tx_buf[idx++] = count + 4; 
    tx_buf[idx++] = 0x82; // SYNC READ
    tx_buf[idx++] = 0x38; // Addr: Present Pos
    tx_buf[idx++] = 0x06; // Len: 6 Bytes
    
    sum += 0xFE + (count+4) + 0x82 + 0x38 + 0x06;
    
    for(int i=0; i<count; i++) {
        tx_buf[idx++] = ids[i];
        sum += ids[i];
    }
    tx_buf[idx++] = ~sum;
    
    // Clear ORE
    volatile uint32_t tmp;
    tmp = huart->Instance->SR;
    tmp = huart->Instance->DR;
    (void)tmp;
    
    if (HAL_UART_Transmit(huart, tx_buf, idx, 2) != HAL_OK) return -1;
    
    // Receive Stream
    // 每个舵机回包 12 字节
    if (HAL_UART_Receive(huart, rx_buf, count * 12, 10) != HAL_OK) return -1;
    
    return 0;
}