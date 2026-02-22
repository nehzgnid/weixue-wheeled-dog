# Debug Log - STM32 Motor Speed Feedback Issue - 2026-02-15

## 1. 问题表现 (Symptoms)
*   **现象**: 上位机 (Python脚本) 通过 USB 接收到的电机速度反馈始终为 `0.000`，即使电机在转动。
*   **排查过程**:
    *   使用 USART1 打印调试信息，确认底层 `Motor_UART5_RxHandler` 能够正确接收到电机控制板发来的原始字符串 (`RAW_MOTOR: $MSPD:...`)。
    *   在 `Motor.c` 中使用 `strtof` 手动解析字符串，确认解析逻辑正常 (`PARSED_SPEED` 有数值)。
    *   但在 `StartFeedbackTask` 主任务中通过 USB 发送的数据 (`[USB_SEND]`) 始终显示 `MotorStr=0.000...`。
    *   引入接收计数器 `RxCnt` 后，发现计数器停留在 `1`，说明中断接收逻辑在处理完第一包数据后就停止了工作。

## 2. 问题原因 (Root Cause)

本次故障由两个核心问题叠加导致：

### A. 编译器优化导致的数据不可见 (Visibility / Caching)
*   **描述**: `Motors` 数组在 UART5 中断服务函数 (ISR) 中被修改，但在 FreeRTOS 的 `StartFeedbackTask` 任务中被读取。
*   **机理**: 由于 `Motors` 定义时未使用 `volatile` 关键字，编译器认为主任务循环中没有修改该变量的代码，为了优化性能，将变量的初始值（0）缓存到了 CPU 寄存器中，而不再每次从内存读取最新值。导致主任务永远“看”不到中断更新的数据。

### B. 中断接收逻辑造成的死锁/溢出 (Interrupt Logic Deadlock / ORE)
*   **描述**: 原始代码采用“半同步”逻辑：收到一包数据后在 ISR 中关闭中断 (`DISABLE_RX_INT`)，等待主循环处理后再重新开启。
*   **机理**:
    1.  **时序不匹配**: 电机发送频率高（例如 10ms/次），而 FreeRTOS 任务调度可能有延迟（几毫秒）。
    2.  **溢出错误 (Overrun)**: 当 STM32 关闭中断期间，电机紧接着发送下一包数据。硬件仍会接收数据移入寄存器，但因中断关闭 CPU 未读取。当新的字节到来时，前一个字节未被读走，触发 **ORE (Overrun Error)** 标志。
    3.  **死锁**: ORE 标志一旦置位，通常会阻塞后续的接收，导致中断服务函数失效，或者主循环重启中断的时机已经错过了数据帧头，导致通信链路彻底卡死（表现为 `RxCnt` 不再增加）。

## 3. 解决方案 (Solution)

### A. 强制内存读取
*   在 `Core/Inc/Motor.h` 和 `Core/Src/Motor.c` 中，将全局变量声明修改为 `volatile`：
    ```c
    extern volatile Motor_TypeDef Motors[MOTOR_COUNT];
    ```

### B. 改为持续接收模式 (Continuous Reception)
*   **修改中断逻辑**: 在 `Motor_UART5_RxHandler` 中，收到完整包（`#`）后，**不再关闭中断**。
*   **流水线处理**: 收到包后立即重置缓冲区索引 (`RxIndex = 0`)，让硬件中断保持开启状态，源源不断地接收后续数据。
*   **解耦**: 主循环不再负责开关中断，仅负责定期检查超时（Watchdog）。

### C. 调试手段优化
*   **移除 ISR 打印**: 删除了中断函数中耗时的 `printf` 操作，避免因打印阻塞导致丢包。
*   **主循环监控**: 在主任务中每 100ms 打印一次接收计数器 `RxCnt` 和原始数据缓存，安全地监控通信状态。

## 4. 关键代码变更
```c
// Core/Src/Motor.c - 优化后的中断处理
void Motor_UART5_RxHandler(void) {
    // ... 读取数据 ...
    if (data == '#') {
        RxBuffer[RxIndex] = '\0';
        Motor_Process_Packet((const char*)RxBuffer);
        
        // 关键修正：不关闭中断，直接重置索引准备下一包
        RxIndex = 0; 
        LastPacketTime = HAL_GetTick();
    }
    // ...
}
```

# Debug Log - STM32 UART5 DMA/IDLE Reception & Initialization Order - 2026-02-15

## 1. 问题表现 (Symptoms)
*   **现象**: 在将 UART5 接收方式从“单字节中断”优化为 **DMA Circular Mode + IDLE Interrupt** 后，系统完全接收不到数据。
*   **观测数据**:
    *   通过 debug 打印的统计数据 `[Motor] I:0 E:0 P:0` 显示 IDLE 中断次数 (I)、错误次数 (E)、数据包解析数 (P) 均为 0。
    *   UART5 中断服务函数 `UART5_IRQHandler` 似乎根本没有进入。
    *   DMA 的流状态始终未启动或立即停止。

## 2. 问题排查 (Troubleshooting)

### A. 中断优先级与向量表检查
*   **猜测**: 可能是 FreeRTOS 对中断优先级的管理屏蔽了 UART5 中断（优先级过高或过低）。
*   **验证**: 检查 NVIC 设置，确认 UART5 优先级处于 FreeRTOS 管理范围内（Priority >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY），配置无误。

### B. 初始化顺序 (Initialization Order)
*   **猜测**: CubeMX 生成代码中 `MX_DMA_Init()` 必须在 `MX_UART5_Init()` 之前调用。
*   **验证**: 检查 `main.c`，顺序正确。

### C. 硬件连接与启动时序 (Hardware & Startup Timing)
*   **关键发现**: 在 `Motor.c` 的 `Motor_Init` 函数中，直接启动 DMA+IDLE 接收可能因为**上电瞬间硬件未就绪**或**线路电气特性不稳定**导致初始化失败。
*   **验证手段**: 在 DMA 初始化之前，增加了一段 **阻塞式硬件自检 (Blocking Hardware Check)** 代码：
    *   使用 `HAL_UART_Receive(&huart5, ...)` 尝试阻塞接收 10 字节数据。
    *   **结果**: 上电自检成功，成功接收到数据。
    *   **意外收获**: 这段阻塞自检代码产生的延时（约 100ms）和对 UART 状态寄存器的“预热”操作，使得随后的 DMA 初始化能够在一个更稳定的硬件状态下执行。

## 3. 解决方案 (Solution)

### A. 增加启动延时与自检
*   在 `Motor_Init` 中，在调用 `HAL_UART_Receive_DMA` 之前，先执行一次短时间的阻塞接收测试。
*   **作用**:
    1.  **滤除上电毛刺**: 消耗掉上电初期可能存在的垃圾数据。
    2.  **确认硬件就绪**: 确保电机驱动板已经开始发送数据，再启动 DMA 监听，避免 DMA 在无数据时空转或因初始噪声进入错误状态。

### B. 关键代码片段 (Core/Src/Motor.c)
```c
void Motor_Init(void) {
    // ... 发送初始化指令 ...
    
    // --- 新增：硬件连接自检与预热 ---
    uint8_t test_buf[32] = {0};
    // 尝试阻塞接收，既是检测也是延时
    HAL_UART_Receive(&huart5, test_buf, 10, 100); 
    
    // ... 随后再启动 DMA ...
    HAL_UART_Receive_DMA(&huart5, DMA_RxBuffer, DMA_RX_BUF_SIZE);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
}
```

### C. 验证结果
*   加入自检代码后，系统启动后串口1输出 `[Motor Check] HARDWARE OK!`。
*   随后 DMA 成功启动，IDLE 中断计数器 `I` 开始增加，电机速度反馈数据 `P` 恢复正常刷新。

## 4. 经验总结
*   在使用 DMA + IDLE 接收时，尽量不要在系统复位后立即（0ms）开启接收，建议在初始化序列中加入适当的延时或握手检测。
*   若遇到 "DMA 不工作" 或 "中断不进入" 的死局，先用最简单的 `HAL_UART_Receive` 阻塞模式测试硬件，往往能快速定位是软件配置问题还是硬件/时序问题。

# Debug Log - STM32 FreeRTOS USB Interrupt Panic - 2026-02-16

## 1. 问题表现 (Symptoms)
*   **现象**: 系统正常启动，但当通过 USB 接口（CDC）向 STM32 发送任意数据时，STM32 立即由运行态转入死机/卡死状态。
*   **观测现象**:
    *   心跳 LED 停止闪烁。
    *   无法再响应任何后续操作（包括电机控制、串口打印）。
    *   不发送 USB 数据时，系统运行正常（只有电机反馈数据上报）。

## 2. 问题排查 (Troubleshooting)

### A. 关键定位
*   问题精准复现于 **USB 接收数据时刻**。
*   接收链路: `CDC_Receive_FS` (ISR) -> `USB_RingBuffer_Push` -> `osThreadFlagsSet`。
*   推测是中断上下文中使用 FreeRTOS API 导致的非法操作。

### B. FreeRTOS 规则检查
*   **FreeRTOS 机制**: 在 Cortex-M 内核上，任何调用 "FromISR" 后缀 API（如 `osThreadFlagsSet`）的中断服务函数，其逻辑中断优先级必须**低于或等于** `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`。
*   **配置检查**:
    *   `FreeRTOSConfig.h`: 系统调用阈值为 **5**。
    *   `usbd_conf.c` (旧): USB OTG FS 根据 CubeMX 默认配置，中断优先为 **0**。
*   **结论**: **0 (USB) < 5 (Threshold)**。高优先级中断非法调用系统 API，导致内核断言 (Assert) 触发死机保护。

### C. 次要问题发现
*   在 `freertos.c` 的 `StartServoTask` 中发现冗余的 `MX_USB_DEVICE_Init()` 调用。该函数已在 `main.c` 中调用，重复初始化可能导致硬件状态寄存器复位或死锁。

## 3. 解决方案 (Solution)

### A. 调整中断优先级
*   修改 `USB_DEVICE/Target/usbd_conf.c`，将 USB OTG FS 的中断优先级降低至安全范围（建议 6~15）。
    ```c
    // 旧配置: HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0); // 危险！
    // 新配置: Must be > 5 for FreeRTOS
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 6, 0); 
    ```

### B. 移除重复初始化
*   注释掉 `Core/Src/freertos.c` 任务中的 `MX_USB_DEVICE_Init()`，确保硬件只在启动时初始化一次。

## 4. 结果 (Status)
*   **Resolved**: 重新编译后，USB 接收不再导致死机。系统能够稳定接收上位机指令并转发给电机，同时保持心跳 LED 正常闪烁。

# Debug Log - Motor Packet Parsing Mismatch (Garbage Prefix) - 2026-02-16

## 1. 问题表现 (Symptoms)
*   **现象**: USB 上报的电机速度数据状态异常：部分电机数据为 0 且始终不更新，个别电机数据固定在某一个旧值（如 10.790），不随电机实际转动变化。
*   **对比**: 
    *   STM32 USB上报: `0.000, 0.000, 10.790, 0.000` (静止不变)
    *   电机板原始数据 (逻辑分析仪/串口透传): `[收] ◆$MSPD:474.69,0.00,10.79,0.00#`
*   **关键特征**: 原始数据包头部存在非 ASCII 字符（乱码/噪声）`◆`。

## 2. 问题排查 (Root Cause)

### A. 刚性帧头匹配失败
*   **代码逻辑**: 原 `RxHandler` 仅简单地将接收字符填入缓冲区，依靠 `#` 触发解析。
    *   填入后的缓冲区内容: `◆$MSPD:474.69...`
*   **解析失败**: `Motor_Process_Packet` 使用 `strncmp(buf, "$MSPD:", 6)` 进行校验。
    *   `strncmp` 要求字符串必须**以 $ 开头**。
    *   由于头部乱码的存在，`strncmp` 返回不相等，导致整包数据被当作无效指令丢弃。
*   **为何有旧值?**: 可能是系统运行过程中偶然有一包数据没有乱码，成功解析了一次，之后一直失败，导致 `Motors` 数组中的值一直保持在最后一次成功更新的状态。

## 3. 解决方案 (Solution)

通过增强代码的鲁棒性来容忍串口噪声：

### A. 接收侧：强制帧头对齐
*   在 DMA 环形缓冲区处理函数 `Process_RingBuffer` 中，一旦检测到起始符 `$`，强制重置行索引 `line_idx = 0`。
*   **效果**: 自动丢弃 `$` 之前的所有乱码字节，确保送入解析器的字符串始终以 `$` 开头。

### B. 解析侧：模糊搜索与容错
*   **搜索策略**: 将 `strncmp` 替换为 `strstr(packet, "$MSPD:")`。
    *   即使缓冲区开头仍有残留无关字符，只要字符串中包含 `$MSPD:` 关键字，就能被定位到。
*   **数字解析**: 在调用 `strtof` 之前，增加逻辑跳过数字前的非数字字符（如冒号、空格），防止解析器因读取到非法字符如 `:` 而直接返回 0.0。

## 4. 关键代码变更 (Core/Src/Motor.c)
```c
// 1. 接收对齐
if (ch == '$') {
    line_idx = 0; // 强制复位，丢弃乱码
}

// 2. 模糊匹配
char* start_ptr = strstr(packet, "$MSPD:");
if (start_ptr != NULL) {
    // ... 解析逻辑 ...
}
```

## 5. 结果 (Status)
*   **Resolved**: 即使原始数据包带有乱码前缀，STM32 也能正确锁定并解析速度数据。USB 上报的数据现在能实时反映电机转速。

# Debug Log - Motor Control Command Failure (DMA Lockup & Parsing Strictness) - 2026-02-16

## 1. 问题表现 (Symptoms)
*   **现象**: 上位机通过 USB 发送 `$pwm:2000,200,200,200#` 指令，STM32 能收到指令（系统无卡死），但电机没有任何动作。
*   **观测现象**: 
    *   COM23 正常打印 `[CMD] $pwm:...`，说明 USB 接收与任务调度正常。
    *   但电机没有转动，且无错误日志输出。

## 2. 问题排查 (Root Cause)

### A. 解析逻辑过严 (Receiver Rigidity)
*   **原因**: `StartCommRxTask` 中使用 `strncmp(buf, "$pwm:", 5)`。
    *   如果指令中包含空格（如 `$pwm: 200`）或不可见字符，或者仅仅是缓冲区没有复位干净，匹配就会失败。
    *   更严重的是，原代码仅做 `透传` (Pass-through)，如果透传函数本身有问题，就没有备用方案。

### B. DMA 发送通道假死 (DMA Stream Lockup)
*   **原因**: `Motor_SendCmd` 使用直接寄存器操作 DMA Stream 7。
    *   代码中检查 `(dma_stream->CR & DMA_SxCR_EN) != 0`。如果检测到忙，直接 `return` (丢弃指令)。
    *   **关键点**: 在 UART DMA 发送模式下，如果 DMA 被错误配置为 **Circular Mode** (循环模式)，发送完一次数据后 `EN` 位不会自动清零。
    *   **结果**: 第一条指令发送成功后，`EN` 始终为 1。后续所有指令因为检测到 `EN=1` 而被永久丢弃。

## 3. 解决方案 (Solution)

### A. 双重解析策略
*   **策略**: 在 `freertos.c` 中，如果不匹配透传格式，增加本地 `sscanf` 解析。
    *   即使直接透传失败，STM32 也会尝试解析出数值并直接写入 `Motors[].PWM_Output`，并在下一个控制周期生效。

### B. DMA 状态机修复
*   **强制解除锁定**: 在 `Motor.c` 发送函数中：
    *   **强制关闭循环模式**: `dma_stream->CR &= ~DMA_SxCR_CIRC`。
    *   **死锁检测**: 如果检测到 `EN=1` 但剩余数据量 `NDTR=0`（说明传输其实已经结束了），软件强制清零 `EN` 位，释放通道，而不是盲目丢弃。
    
## 4. 关键代码变更
```c
// Core/Src/Motor.c
void Motor_SendCmd(const char* cmd) {
    // ...
    // 死锁自愈逻辑
    if ((dma_stream->CR & DMA_SxCR_EN) != 0) {
        if (dma_stream->NDTR == 0) { 
            dma_stream->CR &= ~DMA_SxCR_EN; // 强制复位假死通道
            while(dma_stream->CR & DMA_SxCR_EN); 
        } else {
             return; // 真的忙，才丢弃
        }
    }
    // ...
}
```

## 5. 结果 (Status)
*   **Resolved**: 发送 `$pwm:2000...` 指令后，电机立即响应动作。DMA 发送通道不再因状态位残留而锁死。
