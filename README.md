# 轮腿式机器狗底层固件 (RL-Sync Firmware)

本固件专为轮腿式机器人平台（"Wheeled-Dog"）设计，基于 STM32F407VET6 开发。它提供了针对车轮电机和关节舵机的高频同步控制，并针对强化学习 (RL) 应用进行了优化。

## 1. 硬件配置
*   **MCU**: STM32F407VET6
*   **电机**: 4x 直流减速电机 (带磁编码器)
    *   **最大转速**: 约 120 RPM (12V,2A 供电下)
    *   **PWM 范围**: -3600 至 +3600
    *   **控制模式**: 速度闭环控制 (PID + 前馈)
*   **舵机**: 串行总线舵机 (支持最多 18 个)
    *   第一组: UART2 (ID 1-6)
    *   第二组: UART4 (ID 7-18)
*   **IMU**: 板载集成 IMU (数据包含在反馈包中)

## 2. 通信接口
*   **接口**: USB Type-C (虚拟串口 / CDC)
*   **波特率**: 115200 (USB 通信实际上忽略此参数，但建议设置)
*   **数据位**: 8
*   **停止位**: 1
*   **校验位**: 无 (None)
*   **反馈频率**: 100Hz (10ms 周期)

## 3. 通信协议
固件同时支持用于调试的 **人类可读 ASCII 指令** 和用于高速控制的 **紧凑二进制协议**。

### 3.1 ASCII 指令 (调试用)
所有指令以 `#` 结尾。

*   **设置电机 PWM (开环)**:
    ```text
    $pwm:m1,m2,m3,m4#
    ```
    *   示例: `$pwm:1000,-1000,0,0#` (有效范围: -3600 到 3600)

*   **设置电机速度 (闭环)**:
    ```text
    $spd:v1,v2,v3,v4#
    ```
    *   示例: `$spd:50.0,50.0,-20.5,0.0#` (单位: RPM)

### 3.2 二进制协议 (高性能)
所有二进制数据包遵循以下帧结构 (小端序 Little Endian):

| Head1 | Head2 | Type | Len | Data[...] | Checksum |
| :---: | :---: | :---: | :---: | :---: | :---: |
| 0xA5 | 0x5A | 1 Byte | 1 Byte | Len Bytes | 1 Byte |

**校验和算法**: `Sum(Type + Len + Data[0]...Data[Len-1]) & 0xFF` (仅保留低8位)

#### A. 电机控制 (Type: 0x12)
*   **类型码 (Type)**: `0x12`
*   **逻辑**:
    *   如果 `Len == 16`: 将载荷解析为 **4个 float** (单位: RPM)。
    *   否则: 尝试作为 ASCII 字符串解析 (兼容旧版)。
*   **推荐载荷结构 (16 bytes)**:
    ```c
    struct {
        float speed_m1; // RPM, Little Endian
        float speed_m2;
        float speed_m3;
        float speed_m4;
    } __attribute__((packed));
    ```

#### B. 舵机控制 (Type: 0x10)
*   **类型码 (Type)**: `0x10`
*   **载荷结构**:
    ```c
    struct {
        uint8_t count; // 要控制的舵机数量
        struct {
            uint8_t id;      // 舵机ID
            int16_t pos;     // 目标位置
            uint16_t speed;  // 运动时间/速度
            uint8_t acc;     // 加速度
        } servos[count];     // 紧凑排列，每个舵机 6 bytes
    } __attribute__((packed));
    ```

#### C. 全状态反馈 (Type: 0x40) - 100Hz 自动发送
固件会自动以 100Hz 的频率发送此数据包。
*   **类型码 (Type)**: `TYPE_RL_STATE` (0x40)
*   **载荷结构 (按顺序)**:
    1.  **IMU 数据** (36 bytes): `[AccX, AccY, AccZ, GyroX, GyroY, GyroZ, Roll, Pitch, Yaw]` (均为 float)
    2.  **舵机数量** (1 byte): `N`
    3.  **舵机数据** (N * 7 bytes):
        ```c
        struct {
            uint8_t id;
            int16_t pos;
            int16_t speed;
            int16_t load;
        } item;
        ```
    4.  **电机速度字符串**:
        *   以逗号分隔的 ASCII 字符串附加在数据包末尾: `M1,M2,M3,M4` (例如 `"12.500,-10.200,0.000,5.100"`)。上位机需根据 Packet Len 解析此部分。

## 4. 控制回路参数
经过实机调试的最佳参数：
*   **控制频率**: 100Hz (10ms)
*   **PID 参数**:
    *   **Kp (比例)** = 20.0
    *   **Ki (积分)** = 1.0 (积分限幅 2500)
    *   **Kd (微分)** = 0.0
    *   **Kf (前馈)** = 30.0 (Feedforward)

## 5. 开发环境
*   IDE: Keil MDK-ARM / VS Code
*   OS: Windows 10/11
*   Toolchain: ARMCC v5 / v6



