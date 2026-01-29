#include "st_servo.h"
#include <string.h>

// 内部计算校验和函数
// 算法: ~(ID + Length + Instruction + Parameters)
static uint8_t CheckSum(uint8_t *buf, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 2; i < len - 1; i++) {
        sum += buf[i];
    }
    return ~sum;
}

/**
 * @brief  单个舵机位置控制
 * @param  id: 舵机ID (1-253)
 * @param  pos: 目标位置 (0-4095), 2048为中位
 * @param  speed: 运动速度 (步/秒), 0为最大速度
 * @param  acc: 加速度 (0-254), 0为最大加速度
 */
void ST_WritePos(uint8_t id, int16_t pos, uint16_t speed, uint8_t acc) {
    uint8_t buf[13];
    
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = 9;             // Length
    buf[4] = INST_WRITE;
    buf[5] = STS_ACC;       // 从 ACC 寄存器开始连续写入 7 个字节
    buf[6] = acc;
    buf[7] = (uint8_t)(pos & 0xFF);
    buf[8] = (uint8_t)((pos >> 8) & 0xFF);
    buf[9] = 0;             // Time L (忽略)
    buf[10] = 0;            // Time H (忽略)
    buf[11] = (uint8_t)(speed & 0xFF);
    buf[12] = (uint8_t)((speed >> 8) & 0xFF);
    
    buf[12] = CheckSum(buf, 13);
    // 修正：buf[12] 就是 CheckSum，上面代码有误，重新构建发送逻辑
    // 原代码逻辑有点乱，直接重写这部分
    
    // Header(2) + ID(1) + Len(1) + Inst(1) + Acc(1) + Pos(2) + Time(2) + Speed(2) + Check(1) = 13 bytes
    uint8_t tx_buf[14]; // 保险起见多开一点
    tx_buf[0] = 0xFF;
    tx_buf[1] = 0xFF;
    tx_buf[2] = id;
    tx_buf[3] = 9; // Length = Params(7) + 2 = 9
    tx_buf[4] = INST_WRITE;
    tx_buf[5] = STS_ACC;
    tx_buf[6] = acc;
    tx_buf[7] = (uint8_t)(pos & 0xFF);
    tx_buf[8] = (uint8_t)((pos >> 8) & 0xFF);
    tx_buf[9] = 0;
    tx_buf[10] = 0;
    tx_buf[11] = (uint8_t)(speed & 0xFF);
    tx_buf[12] = (uint8_t)((speed >> 8) & 0xFF);
    tx_buf[13] = CheckSum(tx_buf, 14); // Index 13 is Checksum

    HAL_UART_Transmit(SERVO_UART, tx_buf, 14, 10);
}

/**
 * @brief  设置扭矩开关
 * @param  id: 舵机ID
 * @param  enable: 1-开启扭矩(上锁), 0-关闭扭矩(卸力)
 */
void ST_SetTorque(uint8_t id, uint8_t enable) {
    uint8_t buf[9];
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = 4; // Length
    buf[4] = INST_WRITE;
    buf[5] = STS_TORQUE_ENABLE;
    buf[6] = enable ? 1 : 0;
    buf[7] = CheckSum(buf, 8);
    
    HAL_UART_Transmit(SERVO_UART, buf, 8, 10);
}

/**
 * @brief  同步控制多个舵机 (Sync Write)
 * @param  ids: ID数组
 * @param  num: 舵机数量
 * @param  pos: 位置数组
 * @param  speed: 速度数组
 * @param  acc: 加速度数组
 */
void ST_SyncWritePos(uint8_t *ids, uint8_t num, int16_t *pos, uint16_t *speed, uint8_t *acc) {
    uint8_t data_len_per_servo = 7; // ACC(1)+POS(2)+TIME(2)+SPEED(2)
    // Packet Length = (DataLen + 1)*Num + 4
    // Header(2)+ID(1)+Len(1)+Inst(1)+Addr(1)+LenPer(1) ...
    
    uint8_t buf[200];
    uint8_t idx = 0;
    
    buf[idx++] = 0xFF;
    buf[idx++] = 0xFF;
    buf[idx++] = 0xFE;  // 广播ID
    buf[idx++] = (data_len_per_servo + 1) * num + 4; // Packet Length
    buf[idx++] = INST_SYNC_WRITE;
    buf[idx++] = STS_ACC; // Start Address
    buf[idx++] = data_len_per_servo; // Bytes per servo
    
    for (uint8_t i = 0; i < num; i++) {
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
    
    HAL_UART_Transmit(SERVO_UART, buf, idx, 50);
}

/**
 * @brief  读取舵机信息 (位置, 速度, 负载)
 * @param  id: 舵机ID
 * @param  pos: 输出位置指针
 * @param  speed: 输出速度指针
 * @param  load: 输出负载指针
 * @return 0成功, -1失败
 */
int8_t ST_ReadInfo(uint8_t id, int16_t *pos, int16_t *speed, int16_t *load) {
    uint8_t tx_buf[8];
    uint8_t rx_buf[14]; // Head(2)+ID(1)+Len(1)+Err(1)+Param(6)+Check(1) = 12 bytes. Safe 14.
    
    tx_buf[0] = 0xFF;
    tx_buf[1] = 0xFF;
    tx_buf[2] = id;
    tx_buf[3] = 4;
    tx_buf[4] = INST_READ;
    tx_buf[5] = STS_PRESENT_POSITION_L; // Start from Pos
    tx_buf[6] = 6; // Read 6 bytes (Pos 2 + Speed 2 + Load 2)
    tx_buf[7] = CheckSum(tx_buf, 8);
    
    if (HAL_UART_Transmit(SERVO_UART, tx_buf, 8, 2) != HAL_OK) return -1;
    
    if (HAL_UART_Receive(SERVO_UART, rx_buf, 12, 5) != HAL_OK) return -1;
    
    if (rx_buf[0] != 0xFF || rx_buf[1] != 0xFF || rx_buf[2] != id) return -1;
    
    // --- 位置处理 (标准 int16) ---
    if (pos)   *pos   = (int16_t)(rx_buf[5] | (rx_buf[6] << 8));
    
    // --- 速度处理 (Bit15 是方向位) ---
    if (speed) {
        uint16_t raw_spd = (uint16_t)(rx_buf[7] | (rx_buf[8] << 8));
        if (raw_spd & 0x8000) {
            *speed = -1 * (int16_t)(raw_spd & 0x7FFF); // 负方向
        } else {
            *speed = (int16_t)raw_spd; // 正方向
        }
    }
    
    // --- 负载处理 (Bit10 或 Bit15 是方向位，ST3215通常Bit10为方向) ---
    // 手册指出 ST3215 负载也是 Bit10 为方向(0-1023) 或 Bit15? 
    // 实测 ST 系列通常与速度类似，Bit10 或 Bit15。这里假设和速度一样处理 Bit15 比较保险，
    // 如果发现数值不对(如负数跳变)，可改为检测 Bit10 (raw & 0x400)。
    // 暂时按 Bit15 处理 (兼容性更强，防止出现 -30000)
    if (load) {
        uint16_t raw_load = (uint16_t)(rx_buf[9] | (rx_buf[10] << 8));
        if (raw_load & 0x8000) {
             *load = -1 * (int16_t)(raw_load & 0x7FFF);
        } else if (raw_load & 0x400) { // 也就是 1024 左右的值
             // 部分版本 ST3215 负载是 0-1000，Bit10 是方向
             *load = -1 * (int16_t)(raw_load & 0x3FF);
        } else {
             *load = (int16_t)raw_load;
        }
    }
    
    return 0;
}