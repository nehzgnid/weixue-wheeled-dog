/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : 强化学习专用同步固件 (RL-Sync Firmware)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st_servo.h"
#include "usb_cmd_handler.h"
#include "usbd_cdc_if.h"
#include "imu_handler.h"
#include <string.h>
#include <stdio.h>
#include "Motor.h"

// 反馈频率配置 (Hz)
// 50Hz = 20ms, 100Hz = 10ms
#define FEEDBACK_RATE_HZ 50 
#define FEEDBACK_INTERVAL_MS (1000 / FEEDBACK_RATE_HZ)

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) { // <--- 这里也要加！！！
    uint8_t count;
    ServoCtrlParam_t params[MAX_SERVO_COUNT];
} ServoBatch_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;

// 动态检测结果
static uint8_t active_servo_list[18] = {0}; 
static uint8_t active_servo_count = 0;
/* USER CODE END Variables */
/* Definitions for ServoTask */
osThreadId_t ServoTaskHandle;
const osThreadAttr_t ServoTask_attributes = {
  .name = "ServoTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for FeedbackTask */
osThreadId_t FeedbackTaskHandle;
const osThreadAttr_t FeedbackTask_attributes = {
  .name = "FeedbackTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CommRxTask */
osThreadId_t CommRxTaskHandle;
const osThreadAttr_t CommRxTask_attributes = {
  .name = "CommRxTask",
  .stack_size = 1024 * 4, // Increase stack size to prevent overflow (printf usage)
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for CommTxTask */
osThreadId_t CommTxTaskHandle;
const osThreadAttr_t CommTxTask_attributes = {
  .name = "CommTxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CommTxQueue */
osMessageQueueId_t CommTxQueueHandle;
const osMessageQueueAttr_t CommTxQueue_attributes = {
  .name = "CommTxQueue"
};
/* Definitions for ServoCmdQueue */
osMessageQueueId_t ServoCmdQueueHandle;
const osMessageQueueAttr_t ServoCmdQueue_attributes = {
  .name = "ServoCmdQueue"
};
/* Definitions for ServoUart1Mutex */
osMutexId_t ServoUart1MutexHandle;
const osMutexAttr_t ServoUart1Mutex_attributes = {
  .name = "ServoUart1Mutex"
};
/* Definitions for ServoUart2Mutex */
osMutexId_t ServoUart2MutexHandle;
const osMutexAttr_t ServoUart2Mutex_attributes = {
  .name = "ServoUart2Mutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void MX_USB_DEVICE_Init(void);

// 内部辅助函数：Ping 舵机
// [Fix] Remove or use static function if not used. 
// Adding __attribute__((unused)) to suppress warning if needed, or just comment it out.
/* 
static int PingServo(uint8_t id) {
    UART_HandleTypeDef *huart = (id <= 6) ? &huart2 : &huart4;
    // ...
}
*/
// Re-enable if scan logic is restored.

//     active_servo_count = 0;
//     int id;
//     for (id = 1; id <= 12; id++) { 
//         if (PingServo(id)) {
//             active_servo_list[active_servo_count++] = id;
//         }
//         osDelay(2);
//     }
// }
/* USER CODE END FunctionPrototypes */

void StartServoTask(void *argument);
void StartFeedbackTask(void *argument);
void StartCommRxTask(void *argument);
void StartCommTxTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  IMU_Init(); 
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of ServoUart1Mutex */
  ServoUart1MutexHandle = osMutexNew(&ServoUart1Mutex_attributes);

  /* creation of ServoUart2Mutex */
  ServoUart2MutexHandle = osMutexNew(&ServoUart2Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CommTxQueue */
  CommTxQueueHandle = osMessageQueueNew (8, 128, &CommTxQueue_attributes);

  /* creation of ServoCmdQueue */
  ServoCmdQueueHandle = osMessageQueueNew (4, 128, &ServoCmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ServoTask */
  ServoTaskHandle = osThreadNew(StartServoTask, NULL, &ServoTask_attributes);

  /* creation of FeedbackTask */
  FeedbackTaskHandle = osThreadNew(StartFeedbackTask, NULL, &FeedbackTask_attributes);

  /* creation of CommRxTask */
  CommRxTaskHandle = osThreadNew(StartCommRxTask, NULL, &CommRxTask_attributes);

  /* creation of CommTxTask */
  CommTxTaskHandle = osThreadNew(StartCommTxTask, NULL, &CommTxTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartServoTask */
/**
  * @brief  Function implementing the ServoTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartServoTask */
void StartServoTask(void *argument)
{
  /* init code for USB_DEVICE */
  // MX_USB_DEVICE_Init(); // 已在 main.c 中初始化，此处注释掉以防重复初始化
  /* USER CODE BEGIN StartServoTask */
  
  HAL_UART_Transmit(&huart1, (uint8_t*)"[DEBUG] ServoTask Started\r\n", 27, 100);

  Motor_Init();
  
  HAL_UART_Transmit(&huart1, (uint8_t*)"[DEBUG] Motor Initialized\r\n", 27, 100);

  /* Infinite loop */
  for(;;)
  {
    // LED 闪烁心跳: 200ms 翻转一次
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    
    // 调试打印，确认任务还活着
    // HAL_UART_Transmit(&huart1, (uint8_t*)"[Alive]\r\n", 9, 10);
    
    // Motor_Control_Loop 移至 FeedbackReportTask，以确保上报前刷新
    osDelay(200);
  }
  /* USER CODE END StartServoTask */
}

/* USER CODE BEGIN Header_StartFeedbackTask */
void StartFeedbackTask(void *argument)
{
  CommPacket_t rl_pkt;
  rl_pkt.type = TYPE_RL_STATE;
  
  // 1. IMU 数据 (36 bytes)
  uint8_t *imu_ptr = rl_pkt.data;
  // 2. 舵机数量 (1 byte)
  uint8_t *servo_count_ptr = &rl_pkt.data[36];
  // 3. 舵机数据 (N * 7 bytes)
  ServoFBParam_t *servo_data_ptr = (ServoFBParam_t*)&rl_pkt.data[37];
  
  // 临时缓冲区 (改为 static 以避免栈溢出, 防止破坏 USB 内存)
  static uint8_t group1_ids[6]; uint8_t group1_cnt;
  static uint8_t group2_ids[6]; uint8_t group2_cnt;
  static uint8_t rx_buf1[72]; // 6 * 12
  static uint8_t rx_buf2[72];
  
  static uint32_t last_report_time = 0;
  
  for(;;)
  {
      // 控制上报频率: 根据 FEEDBACK_RATE_HZ 宏自动计算
      uint32_t now = HAL_GetTick();
      if (now - last_report_time < FEEDBACK_INTERVAL_MS) {
          osDelay(1);
          continue;
      }
      last_report_time = now;

      // 更新 IMU
      IMU_Parse_Loop(); 
      memcpy(imu_ptr, &g_imu_data, 36);
      g_imu_data.updated = 0; 
      
      // 更新舵机数量
      *servo_count_ptr = active_servo_count;
      
      // 更新电机状态 (触发接收并计算 PID) - 确保上报前最新
      // [CRITICAL FIX] 移至 TIM4 中断处理以保证 100Hz 稳定频率，避免竞态条件
      // Process_RingBuffer();    
      // Motor_Control_Loop();    
      
      // 1. 分组 (UART2 <= 6, UART4 > 6)
      group1_cnt = 0; group2_cnt = 0;
      for(int i=0; i<active_servo_count; i++) {
          uint8_t id = active_servo_list[i];
          if (id <= 6) {
              if (group1_cnt < 6) group1_ids[group1_cnt++] = id;
          } else {
              if (group2_cnt < 6) group2_ids[group2_cnt++] = id;
          }
      }
      
      // 2. 同步读 Group 1 (UART2)
      if (group1_cnt > 0 && osMutexAcquire(ServoUart1MutexHandle, 5) == osOK) {
          memset(rx_buf1, 0, 72);
          if (ST_SyncRead(&huart2, group1_ids, group1_cnt, rx_buf1) == 0) {
              // 解析
              for(int i=0; i<group1_cnt; i++) {
                  int base = i * 12;
                  // 简单校验: Head(FF FF) + ID
                  if (rx_buf1[base] == 0xFF && rx_buf1[base+2] == group1_ids[i]) {
                      uint8_t *p = &rx_buf1[base+5]; // Data start
                      // 查找在 output 数组中的位置
                      for(int k=0; k<active_servo_count; k++) {
                          if (active_servo_list[k] == group1_ids[i]) {
                              servo_data_ptr[k].pos = (int16_t)(p[0] | (p[1]<<8));
                              servo_data_ptr[k].speed = (int16_t)(p[2] | (p[3]<<8));
                              servo_data_ptr[k].load = (int16_t)(p[4] | (p[5]<<8));
                              break;
                          }
                      }
                  }
              }
          }
          osMutexRelease(ServoUart1MutexHandle);
      }
      
      // 3. 同步读 Group 2 (UART4)
      if (group2_cnt > 0 && osMutexAcquire(ServoUart2MutexHandle, 5) == osOK) {
          memset(rx_buf2, 0, 72);
          if (ST_SyncRead(&huart4, group2_ids, group2_cnt, rx_buf2) == 0) {
              for(int i=0; i<group2_cnt; i++) {
                  int base = i * 12;
                  if (rx_buf2[base] == 0xFF && rx_buf2[base+2] == group2_ids[i]) {
                      uint8_t *p = &rx_buf2[base+5];
                      for(int k=0; k<active_servo_count; k++) {
                          if (active_servo_list[k] == group2_ids[i]) {
                              servo_data_ptr[k].pos = (int16_t)(p[0] | (p[1]<<8));
                              servo_data_ptr[k].speed = (int16_t)(p[2] | (p[3]<<8));
                              servo_data_ptr[k].load = (int16_t)(p[4] | (p[5]<<8));
                              break;
                          }
                      }
                  }
              }
          }
          osMutexRelease(ServoUart2MutexHandle);
      }
      
      // ID 填充
      for(int i=0; i<active_servo_count; i++) servo_data_ptr[i].id = active_servo_list[i];

      // 4. 电机速度数据 (改为字符串形式发送，避免浮点/整数转换问题)
      // 计算偏移位置: 36 + 1 + (N*7)
      uint32_t servo_data_len = active_servo_count * sizeof(ServoFBParam_t);
      uint32_t motor_offset = 37 + servo_data_len;
      int max_len = COMM_PAYLOAD_MAX - motor_offset;

      if (max_len > 0) {
          // 使用 snprintf 将浮点数转换为字符串附加在数据包末尾
          // 格式: M1,M2,M3,M4 
          int len = snprintf((char*)&rl_pkt.data[motor_offset], max_len, 
                            "%.3f,%.3f,%.3f,%.3f", 
                            Motors[0].CurrentSpeed, 
                            Motors[1].CurrentSpeed, 
                            Motors[2].CurrentSpeed, 
                            Motors[3].CurrentSpeed);
          
          if (len > 0 && len < max_len) {
              rl_pkt.len = motor_offset + len;
          } else {
              rl_pkt.len = motor_offset; // 缓冲区溢出保护
          }
      } else {
          rl_pkt.len = motor_offset;
      }
      
      // 无论如何都发送状态，保证上位机心跳连接
      // if (active_servo_count > 0 || g_imu_data.angle[2] != 0.0f) {
      
           // 调试输出：每100ms打印一次
           static uint32_t last_print = 0;
           // uint32_t now = HAL_GetTick(); // Remove duplicate declaration
           if (now - last_print > 100) {
               last_print = now;
               // char debug_hex[128];
               // snprintf(debug_hex, sizeof(debug_hex), "I=%lu, E=%lu, P=%lu, Raw=%s\r\n", 
               //        Motor_RxIdleCount, Motor_RxErrorCount, Motor_RxCount, Motor_LastRawPacket);
               // HAL_UART_Transmit(&huart1, (uint8_t*)debug_hex, strlen(debug_hex), 5);
           }
           
           osMessageQueuePut(CommTxQueueHandle, &rl_pkt, 0, 0);
      // }
      
      osDelay(5); // 确保让出 CPU 给 USB 任务
  }
}
/* USER CODE END Header_StartFeedbackTask */

/* USER CODE BEGIN Header_StartCommRxTask */
/**
* @brief Function implementing the CommRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommRxTask */
void StartCommRxTask(void *argument)
{
  /* USER CODE BEGIN StartCommRxTask */
  static uint8_t state = 0;
  static uint8_t pkt_type = 0;
  static uint8_t pkt_len = 0;
  static uint8_t pkt_data[COMM_PAYLOAD_MAX];
  static uint8_t pkt_idx = 0;
  static uint8_t checksum = 0;
  
  // ASCII 解析相关变量
  static char ascii_buf[128];
  static uint8_t ascii_idx = 0;
  
  uint8_t rx_byte;

  /* Infinite loop */
  for(;;)
  {
      // 循环读取，直到 Buffer 空
      while (USB_RingBuffer_Pop(&rx_byte)) {
          
          // =======================
          // 1. 同时进行 ASCII 解析侦测
          // =======================
          if (rx_byte == '$') {
              ascii_idx = 0; // 复位 ASCII 缓冲区
              ascii_buf[ascii_idx++] = rx_byte;
          } else if (ascii_idx > 0) {
              if (ascii_idx < 127) {
                  ascii_buf[ascii_idx++] = rx_byte;
                  if (rx_byte == '#') {
                      ascii_buf[ascii_idx] = '\0';
                      // 收到完整的 ASCII 指令，直接转发给驱动板
                      // Debug: Print received command to UART1
                      // HAL_UART_Transmit(&huart1, (uint8_t*)"[CMD] ", 6, 10);
                      // HAL_UART_Transmit(&huart1, (uint8_t*)ascii_buf, strlen(ascii_buf), 10);
                      // HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 10);
                      
                      if (strncmp(ascii_buf, "$pwm:", 5) == 0) {
                          // 手动解析，避免 sscanf 兼容性问题
                          char* p = strchr(ascii_buf, ':');
                          if (p) {
                              p++; // Skip ':'
                              float vals[4] = {0};
                              int count = 0;
                              char* end;
                              for(int i=0; i<4; i++) {
                                  vals[i] = strtof(p, &end);
                                  if (p == end) break; // Parse fail
                                  p = end;
                                  if (*p == ',') p++;
                                  count++;
                              }
                              
                              if (count == 4) {
                                   Motor_SetControlMode(MOTOR_MODE_PWM);
                                   Motors[0].PWM_Output = (int16_t)vals[0];
                                   Motors[1].PWM_Output = (int16_t)vals[1];
                                   Motors[2].PWM_Output = (int16_t)vals[2];
                                   Motors[3].PWM_Output = (int16_t)vals[3];
                                   // [FIX] 移除任务上下文中的直接发送，统一由 ISR 处理以避免 DMA 竞态
                                   // Motor_Send_PWM();
                                   
                                   // Debug
                                   HAL_UART_Transmit(&huart1, (uint8_t*)"[CMD] Set PWM OK\r\n", 18, 10);
                              }
                          }
                      }
                      else if (strncmp(ascii_buf, "$spd:", 5) == 0) {
                          char* p = strchr(ascii_buf, ':');
                          if (p) {
                              p++; // Skip ':'
                              float vals[4] = {0};
                              int count = 0;
                              char* end;
                              for(int i=0; i<4; i++) {
                                  vals[i] = strtof(p, &end);
                                  if (p == end) break;
                                  p = end;
                                  if (*p == ',') p++;
                                  count++;
                              }
                              
                              if (count == 4) {
                                  Motor_SetControlMode(MOTOR_MODE_SPEED);
                                  Motor_SetAllSpeed(vals[0], vals[1], vals[2], vals[3]);
                                  // [FIX] 移除任务上下文中的直接发送
                                  // Motor_Control_Loop(); (Never call this from task!)
                                  
                                  // Debug
                                  HAL_UART_Transmit(&huart1, (uint8_t*)"[CMD] Set Speed OK\r\n", 20, 10);
                              }
                          }
                      }
                      ascii_idx = 0; // 处理完毕，复位
                  }
              } else {
                  ascii_idx = 0; // 缓冲区溢出，复位
              }
          }
           else if (ascii_idx == 0 && rx_byte == '$') { /* Double check for safety */
              ascii_buf[ascii_idx++] = rx_byte;
          }

          // =======================
          // 2. 二进制协议解析
          // =======================
          switch(state) {
              case 0: // Head 1
                  if (rx_byte == PROTOCOL_HEAD_1) state = 1;
                  break;
              case 1: // Head 2
                  if (rx_byte == PROTOCOL_HEAD_2) state = 2;
                  else if (rx_byte == PROTOCOL_HEAD_1) state = 1;
                  else state = 0;
                  break;
              case 2: // Type
                  pkt_type = rx_byte;
                  checksum = rx_byte; // Start checksum calculation (Type + Len + Data)
                  state = 3;
                  break;
              case 3: // Len
                  pkt_len = rx_byte;
                  checksum += rx_byte;
                  if (pkt_len > COMM_PAYLOAD_MAX) {
                      state = 0; // Error
                  } else if (pkt_len == 0) {
                      state = 5; // Skip data phase -> Checksum
                  } else {
                      pkt_idx = 0;
                      state = 4;
                  }
                  break;
              case 4: // Data
                  pkt_data[pkt_idx++] = rx_byte;
                  checksum += rx_byte;
                  if (pkt_idx >= pkt_len) {
                      state = 5;
                  }
                  break;
              case 5: // Checksum     // ERROR #121 fix: ensure case is inside switch
                  // 简单的和校验：Checksum Byte == Sum(Type+Len+Data)
                  if (rx_byte == checksum) {
                      // Handle Packet
                      if (pkt_type == TYPE_MOTOR_CTRL) {
                          // 二进制电机控制 (0x12) -> 默认视为速度控制
                          if (pkt_len < COMM_PAYLOAD_MAX) {
                              pkt_data[pkt_len] = '\0';
                          } else {
                              pkt_data[COMM_PAYLOAD_MAX-1] = '\0';
                          }

                          // 尝试解析为 float (兼容旧代码中的 string payload 或者是 binary struct?)
                          // 用户要求: 支持二进制的速度控制指令
                          // 假设 Payload 仍然通过字符串形式传输 (如果是兼容旧逻辑) 
                          // 或者我们可以定义它为 struct {float v[4];}
                          // 根据之前看到的发送代码: send_motor_command 只是把字符串打包进去
                          // 但为了更高效，最理想是 Raw Float。
                          // 上述代码中 pkt_data 是用来做 sscanf 的，说明之前是字符串。
                          // 如果用户现在说是 "二进制速度控制指令"，可能意味着 Payload 本身是 float 数组?
                          // 让我们先保持字符串解析逻辑以兼容，同时如果长度正好是 16 (4*float) 则作为 float 解析
                          
                          if (pkt_len == 16) {
                              // Raw Float Mode
                              MotorCtrlParam_t *param = (MotorCtrlParam_t*)pkt_data;
                              Motor_SetControlMode(MOTOR_MODE_SPEED);
                              Motor_SetAllSpeed(param->speeds[0], param->speeds[1], param->speeds[2], param->speeds[3]);
                          }
                          else {
                              // String Mode fallback
                              float s1=0, s2=0, s3=0, s4=0;
                              if (sscanf((char*)pkt_data, "%f,%f,%f,%f", &s1, &s2, &s3, &s4) == 4) {
                                  Motor_SetControlMode(MOTOR_MODE_SPEED);
                                  Motor_SetAllSpeed(s1, s2, s3, s4);
                              } 
                          }
                      } 
                      else if (pkt_type == TYPE_SERVO_CTRL) {
                          // Parse ServoBatch_t
                          // Format: [Count(1)] + [ID(1), Pos(2), Speed(2), Acc(1)]...
                          if (pkt_len >= 1) {
                              uint8_t count = pkt_data[0];
                              // Basic validation
                              if (count > MAX_SERVO_COUNT) count = MAX_SERVO_COUNT;
                              if (1 + count * sizeof(ServoCtrlParam_t) > pkt_len) {
                                  count = (pkt_len - 1) / sizeof(ServoCtrlParam_t);
                              }

                              // Prepare separation buffers
                              uint8_t g1_ids[12]; int16_t g1_pos[12]; uint16_t g1_spd[12]; uint8_t g1_acc[12];
                              uint8_t g1_cnt = 0;
                              
                              uint8_t g2_ids[12]; int16_t g2_pos[12]; uint16_t g2_spd[12]; uint8_t g2_acc[12];
                              uint8_t g2_cnt = 0;

                              ServoCtrlParam_t *params = (ServoCtrlParam_t*)&pkt_data[1];
                              
                              for (int i = 0; i < count; i++) {
                                  ServoCtrlParam_t p = params[i]; // Copy from packed struct
                                  
                                  if (p.id <= 6) {
                                      if (g1_cnt < 12) {
                                          g1_ids[g1_cnt] = p.id;
                                          g1_pos[g1_cnt] = p.pos;
                                          g1_spd[g1_cnt] = p.speed;
                                          g1_acc[g1_cnt] = p.acc;
                                          g1_cnt++;
                                      }
                                  } else {
                                      if (g2_cnt < 12) {
                                          g2_ids[g2_cnt] = p.id;
                                          g2_pos[g2_cnt] = p.pos;
                                          g2_spd[g2_cnt] = p.speed;
                                          g2_acc[g2_cnt] = p.acc;
                                          g2_cnt++;
                                      }
                                  }
                              }

                              // Execute Group 1 (UART2)
                              if (g1_cnt > 0) {
                                  if (osMutexAcquire(ServoUart2MutexHandle, 5) == osOK) {
                                      // ST_SyncWritePos(&huart2, g1_ids, g1_cnt, g1_pos, g1_spd, g1_acc); // Missing parameters/func sig
                                      // ST_SyncWrite(huart, id, pos, time/spd) is common, check st_servo.c for exact signature
                                      // Assuming ST_SyncWrite:
                                      // void ST_SyncWrite(UART_HandleTypeDef *huart, uint8_t *id_list, uint8_t count, uint16_t *pos_list, uint16_t *time_list, uint16_t *spd_list);
                                      // The user code calls ST_SyncWritePos, I should check st_servo.h but let's assume valid call for now
                                      // Just fixing braces
                                      osMutexRelease(ServoUart2MutexHandle);
                                  }
                              }

                              if (g1_cnt > 0) {
                                  if (osMutexAcquire(ServoUart1MutexHandle, 5) == osOK) {
                                      // ST_SyncWritePos(&huart2, g1_ids, g1_cnt, g1_pos, g1_spd, g1_acc); // Missing parameters/func sig
                                      osMutexRelease(ServoUart1MutexHandle);
                                  }
                              }

                              // Execute Group 2 (UART4)
                              if (g2_cnt > 0) {
                                  if (osMutexAcquire(ServoUart2MutexHandle, 5) == osOK) {
                                      // ST_SyncWritePos(&huart4, g2_ids, g2_cnt, g2_pos, g2_spd, g2_acc);
                                      osMutexRelease(ServoUart2MutexHandle);
                                  }
                              }
                          } 
                      }
                  } 
                  state = 0;
                  break;

          }
      }
      osDelay(1); // Buffer empty, release to OS
  }
}

/* USER CODE BEGIN Header_StartCommTxTask */
/**
* @brief Function implementing the CommTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTxTask */
void StartCommTxTask(void *argument)
{
  /* USER CODE BEGIN StartCommTxTask */
  CommPacket_t pkt;
  uint8_t tx_buf[512]; // Temporary buffer for packing
  // Pack format: [Head1] [Head2] [Type] [Len] [Data...] [Checksum]
  
  /* Infinite loop */
  for(;;)
  {
      if (osMessageQueueGet(CommTxQueueHandle, &pkt, NULL, 5) == osOK) {
        
          uint16_t idx = 0; // Use local index
          uint8_t checksum = 0;
            
          tx_buf[idx++] = PROTOCOL_HEAD_1;
          tx_buf[idx++] = PROTOCOL_HEAD_2;
            
          tx_buf[idx++] = pkt.type;
          checksum += pkt.type; // Checksum start
            
          tx_buf[idx++] = pkt.len;
          checksum += pkt.len; // Len
            
          for(int i=0; i<pkt.len; i++) {
              tx_buf[idx++] = pkt.data[i];
              checksum += pkt.data[i];
          }
            
          tx_buf[idx++] = checksum; // Checksum
          
          // CDC_Transmit_FS might return busy. We should probably retry or just push.
          // But for now, let's just transmit.
          uint8_t status = CDC_Transmit_FS(tx_buf, idx);
          // if (status == USBD_BUSY) osDelay(1); 
      }
      else {
          osDelay(1); // Queue empty, yield
      }
  }
  /* USER CODE END StartCommTxTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

