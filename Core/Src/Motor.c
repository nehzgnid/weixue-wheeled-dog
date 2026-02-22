#include "Motor.h"
// Force Rebuild Trigger: Added variable definition
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "FreeRTOS.h" // 引入 FreeRTOS 头文件以使用临界区宏
#include "task.h"

// 引用 DMA 句柄 (CubeMX 生成)
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx; // Add explicit extern for TX handle

// 电机数组
volatile Motor_TypeDef Motors[MOTOR_COUNT];
volatile Motor_ControlMode_TypeDef GlobalControlMode = MOTOR_MODE_PWM; // 默认为开环 PWM
volatile uint8_t Motor_Initialized = 0; // 定义初始化标志位

// UART5 DMA 环形缓冲区配置
#define DMA_RX_BUF_SIZE 256  // 定义足够大的缓冲区，确保不会被 DMA 快速填满
static uint8_t DMA_RxBuffer[DMA_RX_BUF_SIZE];
static uint32_t LastReadPos = 0; // 上次 CPU 读取的位置 (RingBuffer Read Ptr)

// 接收状态标志
static uint32_t LastPacketTime = 0; // 上次接收时间

// PID 参数 (针对单位校准后的 RPM)
#define DEFAULT_KP 20.0f   // 增大比例增益: 误差 10 RPM -> 调整 200 PWM
#define DEFAULT_KI 1.0f    // 增大积分增益，消除稳态误差
#define DEFAULT_KD 0.1f    // 微分
#define DEFAULT_KF 30.0f   // 增大前馈: 假设满速 120 RPM 对应 3600 PWM -> KF=30
#define INTEGRAL_LIMIT 2500.0f // 增大积分限幅
#define OUTPUT_LIMIT 3600.0f

// 私有函数声明
static void Motor_PID_Init(void);
static void Motor_Process_Packet(const char* packet);
static void Motor_Calc_PWM(void);
// void Motor_Send_PWM(void); // Header defined
// static void Process_RingBuffer(void); // Global defined

// 初始化电机模块
void Motor_Init(void) {
    // 调试打印
    HAL_UART_Transmit(&huart1, (uint8_t*)"[Motor] Init Start...\r\n", 23, 100);

    // 默认模式
    GlobalControlMode = MOTOR_MODE_PWM;

    // 1. PID 初始化
    Motor_PID_Init();

    // 2. 配置电机控制模块
    HAL_Delay(500); 
    
    // 发送初始化指令
    // 校准参数修正: 
    // 用户反馈: 指令/反馈显示 2.05 时，实际转速约为 20 RPM。
    // 这说明 2.05 单位并不是 RPM，而是 RPM/10。或者编码器线数配置导致单位错乱。
    // 如果 2.05 = 20 RPM，倍率为 ~10 倍。
    // 我们之前的 $mline:546 可能是对的 (13*42)，但驱动板反馈回来的单位可能不是 RPM，而是 RPS (每秒转数)?
    // 2.05 * 60 = 123 RPM (额定转速附近)。这就对上了！
    // 结论: 反馈回来的单位是 **转/秒 (RPS)**。
    // 为了让 PID 正常工作，我们需要统一单位。
    // 要么上位机发 rps，要么下位机转换。为了方便，我们把 PID 参数针对 RPS 调整，或者把反馈乘60。
    // 简单起见，既然现在的 2.05 对应 20 RPM (等等，2.05*60=123RPM，如果是20RPM那就不对)。
    // 重新计算: 2.05 (反馈值) -> 20 RPM (实际值)。倍率 = 10。
    // 意味着现在的反馈值太小了，要乘 10 才是 RPM。
    // 这可能是因为 $mline 设的 546 太大了? 
    // 如果是 mline=546，驱动以为转一圈要546脉冲。如果实际只要54.6脉冲，那驱动就会算出转速只有实际的1/10。
    // 但物理参数 13*42=546 没错。
    // 可能驱动板内部还有个采样时间系数。
    // 不纠结驱动板内部算法，我们直接在 PID 反馈环节修正这个系数。
    // 假设反馈值是 F, 真实 RPM = F * 10.
    // 我们在 Process_Packet 中把解析出来的 CurrentSpeed * 10。
    
    Motor_SendCmd("$stop#"); HAL_Delay(100);
    Motor_SendCmd("$mtype:3#"); HAL_Delay(100);
    Motor_SendCmd("$mline:546#"); HAL_Delay(100); 
    Motor_SendCmd("$wdiameter:75#"); HAL_Delay(100);
    Motor_SendCmd("$deadzone:100#"); HAL_Delay(100);
    Motor_SendCmd("$upload:0,0,1#"); HAL_Delay(10);
    
    // 强制发送一次0速PWM，防止电机失控
    Motor_SendCmd("$pwm:0,0,0,0#"); HAL_Delay(10);
    
    // --- 硬件连接自检 (Hardware Check) ---
    // 尝试阻塞接收 10 个字节，超时 100ms
    // 如果这里能收到数据，说明硬件 TX/RX 连接正常
    uint8_t test_buf[32] = {0};
    HAL_StatusTypeDef test_status = HAL_UART_Receive(&huart5, test_buf, 10, 100);
    
    char log_buf[128];
    if (test_status == HAL_OK || test_status == HAL_TIMEOUT) { 
        // 即使 TIMEOUT，如果收到了部分数据(RxXferCount < 10)，也说明线路是通的
        // HAL_UART_Receive 在超时时会返回 HAL_TIMEOUT，但可以通过 huart5.RxXferCount 查看剩余未接收字节
        // 实际接收数 = 请求数 - 剩余数
        int received_count = 10 - huart5.RxXferCount;
         
        if (received_count > 0 || test_status == HAL_OK) {
             snprintf(log_buf, sizeof(log_buf), "[Motor Check] HARDWARE OK! Recv %d bytes: %02X %02X...\r\n", 
                      (test_status==HAL_OK)?10:received_count, test_buf[0], test_buf[1]);
        } else {
             snprintf(log_buf, sizeof(log_buf), "[Motor Check] NO DATA received (Timeout). Check wiring!\r\n");
        }
    } else {
        snprintf(log_buf, sizeof(log_buf), "[Motor Check] Error: %d\r\n", test_status);
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen(log_buf), 100);
    // --- 自检结束 ---

    // 3. 开启 DMA 接收 (Circular Mode)并启用 IDLE 中断
    // 启动 DMA 循环接收，数据直接搬运到 DMA_RxBuffer
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(&huart5, DMA_RxBuffer, DMA_RX_BUF_SIZE);
    
    // 调试 DMA 启动状态
    snprintf(log_buf, sizeof(log_buf), "[Motor] DMA Start: %s\r\n", status == HAL_OK ? "OK" : "FAIL");
    HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen(log_buf), 100);

    // 开启 IDLE (空闲) 中断：强制操作寄存器
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
    // 额外保险：确保 CR1 中的 IDLEIE 位确实被置位了
    if ((UART5->CR1 & USART_CR1_IDLEIE) == 0) {
        UART5->CR1 |= USART_CR1_IDLEIE;
        HAL_UART_Transmit(&huart1, (uint8_t*)"[Motor] IDLE Force Enabled\r\n", 27, 100);
    }
    
    Motor_Initialized = 1; // 标记初始化完成
}

// 初始化PID参数
void Motor_PID_Init(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        Motors[i].PID.Kp = DEFAULT_KP;
        Motors[i].PID.Ki = DEFAULT_KI;
        Motors[i].PID.Kd = DEFAULT_KD;
        Motors[i].PID.Kf = DEFAULT_KF;
        Motors[i].PID.IntegralLimit = INTEGRAL_LIMIT;
        Motors[i].PID.OutputLimit = OUTPUT_LIMIT;
        
        Motors[i].PID.Error = 0.0f;
        Motors[i].PID.LastError = 0.0f;
        Motors[i].PID.Integral = 0.0f;
        
        Motors[i].TargetSpeed = 0.0f;
        Motors[i].CurrentSpeed = 0.0f;
        Motors[i].PWM_Output = 0;
    }
}

// 设置控制模式
void Motor_SetControlMode(Motor_ControlMode_TypeDef mode) {
    GlobalControlMode = mode;
    // 如果切换到速度模式，清除 PID 积分和误差，防止瞬时跳变
    if (mode == MOTOR_MODE_SPEED) {
        for (int i = 0; i < MOTOR_COUNT; i++) {
            Motors[i].PID.Integral = 0;
            Motors[i].PID.LastError = 0;
        }
    }
}

// 设置单个电机目标速度 (仅在 PID 模式下有意义，但也更新 TargetSpeed 变量)
void Motor_SetSpeed(uint8_t motor_index, float speed) {
    if (motor_index < MOTOR_COUNT) Motors[motor_index].TargetSpeed = speed;
}

// 设置所有电机目标速度
void Motor_SetAllSpeed(float speed1, float speed2, float speed3, float speed4) {
    Motors[0].TargetSpeed = speed1;
    Motors[1].TargetSpeed = speed2;
    Motors[2].TargetSpeed = speed3;
    Motors[3].TargetSpeed = speed4;
}

// 控制循环
void Motor_Control_Loop(void) {
    // 仅在速度闭环模式下进行 PID 计算
    if (GlobalControlMode == MOTOR_MODE_SPEED) {
        // 1. PID 计算
        Motor_Calc_PWM();
    }
    
    // 2. 发送控制指令 (无论什么模式，都需要发送计算出或设置好的 PWM_Output)
    Motor_Send_PWM();
    
    // 注意：数据接收现在由 DMA + IDLE 中断自动处理，主循环无需干预接收逻辑
}

// 调试变量
volatile uint32_t Motor_RxCount = 0;
volatile uint32_t Motor_RxIdleCount = 0; 
volatile uint32_t Motor_RxErrorCount = 0; // 新增：错误计数器
char Motor_LastRawPacket[64] = {0};

// UART5 中断处理函数 - 改为处理 IDLE 中断
void Motor_UART5_RxHandler(void) {
    uint32_t isr_flags = UART5->SR;
    uint32_t cr1_flags = UART5->CR1;

    // 1. 检查错误标志 (ORE=Overrun, NE=Noise, FE=Frame)
    if (isr_flags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) {
        // 错误处理：清除标志
        volatile uint32_t tmpreg; 
        tmpreg = UART5->SR;
        tmpreg = UART5->DR;
        (void)tmpreg;
        Motor_RxErrorCount++;
    }

    // 2. 检查 IDLE 标志
    if (((isr_flags & USART_SR_IDLE) != RESET) && ((cr1_flags & USART_CR1_IDLEIE) != RESET)) {
        
        // 清除 IDLE 标志 (F4 标准: 读SR -> 读DR)
        volatile uint32_t tmpreg; 
        tmpreg = UART5->SR;
        tmpreg = UART5->DR;
        (void)tmpreg; 
        
        // 关键修改：中断中不再直接解析数据，只设置标志位或不处理
        // IDLE中断主要用于唤醒或通知，具体的 RingBuffer 处理交给主循环（StartFeedbackTask）
        // 这样避免了 ISR 和 Task 同时操作 RingBuffer 的竞态条件
        // Process_RingBuffer();  <-- 移除此行
        
        LastPacketTime = HAL_GetTick();
        Motor_RxIdleCount++;
    }
}

// ================== DMA 环形缓冲区处理核心逻辑 ==================

// 临时行缓冲区，用于解析单行指令
static char line_buffer[128];
static uint8_t line_idx = 0;

void Process_RingBuffer(void) {
    // 1. 获取 DMA 当前写入位置 (Write Ptr)
    // CNDTR 是倒计数的 (Remaining Data items)，所以：
    // WritePos = BufferSize - CNDTR
    uint32_t WritePos = DMA_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart5_rx);
    
    // 如果读写指针相等，说明没新数据 (或恰好跑了一圈，小概率)
    if (WritePos == LastReadPos) return;
    
    // 2. 遍历新收到的数据
    while (LastReadPos != WritePos) {
        uint8_t ch = DMA_RxBuffer[LastReadPos];
        
        // 处理单字节：拼凑成行
        if (ch == '$') {
            line_idx = 0; // 强制复位，以 $ 开头
        }
        
        if (line_idx < 127) {
            line_buffer[line_idx++] = ch;
            
            // 检查结束符 '#'
            if (ch == '#') {
                line_buffer[line_idx] = '\0'; // 字符串结束符
                
                // 处理完整的一行
                Motor_Process_Packet(line_buffer);
                
                // 重置行缓冲区
                line_idx = 0;
            }
        } else {
            // 行溢出，强制重置
            line_idx = 0;
        }
        
        // 移动读指针 (环形回绕)
        LastReadPos++;
        if (LastReadPos >= DMA_RX_BUF_SIZE) {
            LastReadPos = 0;
        }
    }
}
// ==============================================================

// 调试变量
// volatile uint32_t Motor_RxCount = 0;
// volatile uint32_t Motor_RxIdleCount = 0; // 新增：IDLE中断计数器
// char Motor_LastRawPacket[64] = {0};

// 处理接收到的数据包
static void Motor_Process_Packet(const char* packet) {
    // 1. 复制数据到全局缓冲区供主任务查看
    strncpy(Motor_LastRawPacket, packet, 63);
    Motor_LastRawPacket[63] = '\0';
    Motor_RxCount++; 


    // 检查是否为速度数据 "$MSPD:..."
    // 允许前面有少许乱码，只要包含 $MSPD即可
    char* start_ptr = strstr(packet, "$MSPD:");
    if (start_ptr != NULL) {
        char* values_start = start_ptr + 6;
        char temp_buffer[64];
        strncpy(temp_buffer, values_start, sizeof(temp_buffer));
        temp_buffer[sizeof(temp_buffer)-1] = '\0';
        
        // 查找结束符并截断
        char* end_ptr = strchr(temp_buffer, '#');
        if (end_ptr) *end_ptr = '\0';
        
        // 移除可能的首部空白字符或乱码 (直到找到数字或负号)
        char* clean_start = temp_buffer;
        while (*clean_start && !((*clean_start >= '0' && *clean_start <= '9') || *clean_start == '-')) {
            clean_start++; 
        }

        // 分割并解析 (手动分割)
        int idx = 0;
        char* ptr = clean_start;
        char* token_start = ptr;
        
        while (*ptr != '\0' && idx < MOTOR_COUNT) {
            if (*ptr == ',') {
                *ptr = '\0'; // 替换为结束符
                // 修正反馈速度单位: 之前反馈 2.05 对应 20 RPM，倍率为 10
                Motors[idx].CurrentSpeed = strtof(token_start, NULL) * 10.0f;
                idx++;
                token_start = ptr + 1;
            }
            ptr++;
        }
        // 处理最后一个值
        if (idx < MOTOR_COUNT && *token_start != '\0') {
            Motors[idx].CurrentSpeed = strtof(token_start, NULL) * 10.0f;
        }
    }
}

// PID 计算和前馈
static void Motor_Calc_PWM(void) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        // 目标与当前误差
        float error = Motors[i].TargetSpeed - Motors[i].CurrentSpeed;
        Motors[i].PID.Error = error;
        
        // 积分项
        Motors[i].PID.Integral += error;
        // 积分限幅
        if (Motors[i].PID.Integral > Motors[i].PID.IntegralLimit)
            Motors[i].PID.Integral = Motors[i].PID.IntegralLimit;
        else if (Motors[i].PID.Integral < -Motors[i].PID.IntegralLimit)
            Motors[i].PID.Integral = -Motors[i].PID.IntegralLimit;
        
        // 微分项
        float derivative = error - Motors[i].PID.LastError;
        Motors[i].PID.LastError = error;
        
        // 计算 PID 输出
        float pid_out = (Motors[i].PID.Kp * error) + 
                        (Motors[i].PID.Ki * Motors[i].PID.Integral) + 
                        (Motors[i].PID.Kd * derivative);
        
        // 前馈项 Feedforward = Kf * TargetSpeed
        float feedforward = Motors[i].PID.Kf * Motors[i].TargetSpeed;
        
        // 总输出
        float total_out = pid_out + feedforward;
        
        // 输出限幅
        if (total_out > Motors[i].PID.OutputLimit) total_out = Motors[i].PID.OutputLimit;
        else if (total_out < -Motors[i].PID.OutputLimit) total_out = -Motors[i].PID.OutputLimit;
        
        Motors[i].PWM_Output = (int16_t)total_out;
    }
}

// 简单的整数转字符串辅助函数
static void append_int(char* buffer, int* pos, int value) {
    if (value == 0) {
        buffer[(*pos)++] = '0';
        return;
    }
    char temp[16];
    int i = 0;
    
    // 处理负数
    if (value < 0) {
        buffer[(*pos)++] = '-';
        // 使用绝对值注意 INT_MIN
        value = -value;
    }
    
    // 数字转换
    while (value > 0) {
        temp[i++] = (value % 10) + '0';
        value /= 10;
    }
    
    // 反转
    while (i > 0) {
        buffer[(*pos)++] = temp[--i];
    }
}

static void append_str(char* buffer, int* pos, const char* str) {
    while (*str) buffer[(*pos)++] = *str++;
}

// 发送 PWM 控制指令 (Global)
void Motor_Send_PWM(void) {
    char cmd_buffer[64];
    int pos = 0;
    
    // 手动构建字符串: "$pwm:M1,M2,M3,M4#"
    append_str(cmd_buffer, &pos, "$pwm:");
    append_int(cmd_buffer, &pos, Motors[0].PWM_Output);
    append_str(cmd_buffer, &pos, ",");
    append_int(cmd_buffer, &pos, Motors[1].PWM_Output);
    append_str(cmd_buffer, &pos, ",");
    append_int(cmd_buffer, &pos, Motors[2].PWM_Output);
    append_str(cmd_buffer, &pos, ",");
    append_int(cmd_buffer, &pos, Motors[3].PWM_Output);
    append_str(cmd_buffer, &pos, "#");
    cmd_buffer[pos] = '\0';
    
    /*
    snprintf(cmd_buffer, sizeof(cmd_buffer), "$pwm:%d,%d,%d,%d#", 
             Motors[0].PWM_Output, 
             Motors[1].PWM_Output, 
             Motors[2].PWM_Output, 
             Motors[3].PWM_Output);
    */
    
    Motor_SendCmd(cmd_buffer);
}

// 发送指令到底层 UART
// 注意：DMA 发送是非阻塞的，所以传入的 buffer 必须在发送期间保持有效。
// 这里我们使用一个静态缓冲区来缓存要发送的数据。
static char dma_tx_buffer[64]; 

void Motor_SendCmd(const char* cmd) {
    size_t len = strlen(cmd);
    if (len > 63) return; 

    // 手动 DMA 发送 (Bypass HAL Lock)
    // 检查 DMA Stream 是否忙 (EN=1 表示忙)
    // 使用 huart5 关联的 hdmatx 句柄获取 instance
    // DMA_Stream_TypeDef *dma_stream = huart5.hdmatx->Instance; // <-- 潜在风险点
    
    // 使用显式的 extern 变量确保安全
    if (hdma_uart5_tx.Instance == NULL) return; // 防御性编程
    DMA_Stream_TypeDef *dma_stream = hdma_uart5_tx.Instance;
    
    // 如果 DMA 还在工作，我们只能丢弃或者等待。这里选择丢弃防止阻塞。
    if ((dma_stream->CR & DMA_SxCR_EN) != 0) {
        // 尝试检查是否是上次传输其实已经完成但 EN 未清零 (例如因 TCIE 没开 TCIF 没处理?)
        // 在 Normal Mode 下，NDTR=0 时自动清零 EN。
        if (dma_stream->NDTR == 0) {
            dma_stream->CR &= ~DMA_SxCR_EN; // 强制关闭
            while(dma_stream->CR & DMA_SxCR_EN); // 等待关闭确认
        } else {
             // 真的还在忙，只能放弃
             return; 
        }
    }

    // 复制数据
    memcpy(dma_tx_buffer, cmd, len);
    dma_tx_buffer[len] = '\0'; 

    // 1. 清除 UART TC 标志 (确保后续检测正确)
    __HAL_UART_CLEAR_FLAG(&huart5, UART_FLAG_TC);

    // 2. 清除 DMA 中断标志 (确保干净启动)
    // 直接操作 DMA1->HIFCR 寄存器，清除 Stream 7 的所有标志 (位 27,26,25,24,22)
    // DMA_FLAG_TCIF3_7 | DMA_FLAG_HTIF3_7 | DMA_FLAG_TEIF3_7 | DMA_FLAG_DMEIF3_7 | DMA_FLAG_FEIF3_7
    // 对应值：0x0F400000 | 0x00400000 = 0x0F800000 ? No.
    // TCIF7=bit27, HTIF7=bit26, TEIF7=bit25, DMEIF7=bit24, FEIF7=bit22
    // (1<<27)|(1<<26)|(1<<25)|(1<<24)|(1<<22) = 0x0F400000
    DMA1->HIFCR = (uint32_t)(0x0F400000 | 0x00400000); 

    // 3. 配置 DMA 地址和长度
    dma_stream->M0AR = (uint32_t)dma_tx_buffer; // Memory Addr
    dma_stream->PAR  = (uint32_t)&huart5.Instance->DR; // Periph Addr
    dma_stream->NDTR = (uint32_t)len;       // Length

    // 确保关闭 Circular Mode (循环模式会导致 EN 永不自动清除)
    dma_stream->CR &= ~DMA_SxCR_CIRC;

    // 4. 确保关闭 DMA 中断 (TCIE, HTIE, TEIE, DMEIE)
    // 我们不需要中断通知 (Fire-and-Forget)，且防止 HAL_DMA_IRQHandler 导致的死锁
    dma_stream->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
    
    // 5. 启用 DMA Stream
    // 注意：hdma_uart5_tx 在初始化时已经配置好方向、中断等，这里只需要 Enable
    // 但是 HAL_DMA_Init 不会自动开启某些位，我们手动确保 EN 被置位
    SET_BIT(dma_stream->CR, DMA_SxCR_EN);
    
    // Debug: Check if DMA is running
    // if ((dma_stream->CR & DMA_SxCR_EN) == 0) {
    //      // If EN failed to set (e.g. error flag set earlier), we have a problem
    // }

    // 5. 启用 UART 的 DMA 发送请求 (如果还没开)
    SET_BIT(huart5.Instance->CR3, USART_CR3_DMAT);
}
