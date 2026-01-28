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
    
    // 写入校验和
    buf[12] = CheckSum(buf, 13); // 这里是个小bug修正：校验和应该是最后一个字节，原数组大小13，下标12是最后一个
    // 但上面的 buf[12] 已经被 speed 高位占用了，数组开小了？
    // 让我们重新数一下：
    // Header(2) + ID(1) + Len(1) + Inst(1) + Addr(1) + Data(7) + Check(1) = 14字节
    // 抱歉，上面数组定义小了1个字节，逻辑需要微调。
    
    // 重新定义并修正逻辑
    uint8_t tx_buf[14];
    tx_buf[0] = 0xFF;
    tx_buf[1] = 0xFF;
    tx_buf[2] = id;
    tx_buf[3] = 10; // Length = DataLen(7) + 3 (Inst+Addr+Check) = 10? No.
                    // Standard: Length = Parameter Count + 2. Params = Addr(1)+Data(7)=8. So Length=10. Correct.
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
    buf[7] = 0; // 这里的逻辑: Length=4, Params=2(Addr+Val). 
                // 校验和位置应该是 buf[7]
    buf[7] = CheckSum(buf, 8); // 计算前8个字节
    
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
    uint8_t total_len = 7 + (data_len_per_servo + 1) * num + 1; // Header(2)+ID(1)+Len(1)+Inst(1)+Addr(1)+LenPer(1) + ... + Check(1)
    
    // 为了安全，限制最大发送缓冲区，假设最多20个舵机
    // 20 * 8 + 10 = 170 字节左右
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
