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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) { // <--- 这里也要加！！！
    uint8_t count;
    ServoCtrlParam_t params[MAX_SERVO_COUNT];
} ServoBatch_t;
/* USER CODE END PTD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;

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
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh, // 提升优先级
};
/* Definitions for CommTxTask */
osThreadId_t CommTxTaskHandle;
const osThreadAttr_t CommTxTask_attributes = {
  .name = "CommTxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh, // 提升优先级
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
static int PingServo(uint8_t id) {
    UART_HandleTypeDef *huart = (id <= 6) ? &huart2 : &huart4;
    uint8_t tx[6] = {0xFF, 0xFF, id, 0x02, INST_PING, 0};
    uint8_t rx[6];
    
    uint8_t sum = 0;
    int i;
    for(i=2; i<5; i++) sum += tx[i];
    tx[5] = ~sum;
    
    HAL_UART_Transmit(huart, tx, 6, 2);
    if (HAL_UART_Receive(huart, rx, 6, 2) == HAL_OK) {
        if (rx[0]==0xFF && rx[1]==0xFF && rx[2]==id) return 1; 
    }
    return 0; 
}

static void ScanServos(void) {
    active_servo_count = 0;
    int id;
    for (id = 1; id <= 12; id++) { 
        if (PingServo(id)) {
            active_servo_list[active_servo_count++] = id;
        }
        osDelay(2);
    }
}
/* USER CODE END FunctionPrototypes */

void StartServoTask(void *argument);
void StartFeedbackTask(void *argument);
void StartCommRxTask(void *argument);
void StartCommTxTask(void *argument);

void MX_FREERTOS_Init(void); 

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  IMU_Init(); 
  /* USER CODE END Init */

  ServoUart1MutexHandle = osMutexNew(&ServoUart1Mutex_attributes);
  ServoUart2MutexHandle = osMutexNew(&ServoUart2Mutex_attributes);

  CommTxQueueHandle = osMessageQueueNew (8, 128, &CommTxQueue_attributes);
  ServoCmdQueueHandle = osMessageQueueNew (4, 128, &ServoCmdQueue_attributes);

  ServoTaskHandle = osThreadNew(StartServoTask, NULL, &ServoTask_attributes);
  FeedbackTaskHandle = osThreadNew(StartFeedbackTask, NULL, &FeedbackTask_attributes);
  CommRxTaskHandle = osThreadNew(StartCommRxTask, NULL, &CommRxTask_attributes);
  CommTxTaskHandle = osThreadNew(StartCommTxTask, NULL, &CommTxTask_attributes);
}

/* USER CODE BEGIN Header_StartServoTask */
void StartServoTask(void *argument)
{
  osDelay(100);
  MX_USB_DEVICE_Init(); 
  
  ServoBatch_t cmd;
  uint8_t ids1[6], ids2[6];
  int16_t pos1[6], pos2[6];
  uint16_t spd1[6], spd2[6];
  uint16_t acc1[6], acc2[6];

  osDelay(500);
  
  // === 上电自检：恢复自动扫描模式 ===
  if (osMutexAcquire(ServoUart1MutexHandle, osWaitForever) == osOK) {
      if (osMutexAcquire(ServoUart2MutexHandle, osWaitForever) == osOK) {
          ScanServos();
          osMutexRelease(ServoUart2MutexHandle);
      }
      osMutexRelease(ServoUart1MutexHandle);
  }

  // 只对在线的舵机上锁
  int i;
  for(i=0; i < active_servo_count; i++) {
      uint8_t id = active_servo_list[i];
      UART_HandleTypeDef *huart = (id <= 6) ? &huart2 : &huart4;
      osMutexId_t mutex = (id <= 6) ? ServoUart1MutexHandle : ServoUart2MutexHandle;
      
      if (osMutexAcquire(mutex, 10) == osOK) {
          ST_SetTorque(huart, id, 1);
          osMutexRelease(mutex);
      }
      osDelay(5);
  }

  for(;;)
  {
      if (osMessageQueueGet(ServoCmdQueueHandle, &cmd, NULL, osWaitForever) == osOK) {
          uint8_t n1 = 0, n2 = 0;
          int j;
          for(j=0; j<cmd.count; j++) {
              if (cmd.params[j].id <= 6) { 
                  ids1[n1] = cmd.params[j].id;
                  pos1[n1] = cmd.params[j].pos;
                  spd1[n1] = cmd.params[j].speed;
                  acc1[n1] = cmd.params[j].acc;
                  n1++;
              } else { 
                  ids2[n2] = cmd.params[j].id;
                  pos2[n2] = cmd.params[j].pos;
                  spd2[n2] = cmd.params[j].speed;
                  acc2[n2] = cmd.params[j].acc;
                  n2++;
              }
          }
          
          if (n1 > 0 && osMutexAcquire(ServoUart1MutexHandle, 5) == osOK) {
              ST_SyncWritePos(&huart2, ids1, n1, pos1, spd1, acc1);
              osMutexRelease(ServoUart1MutexHandle);
          }
          if (n2 > 0 && osMutexAcquire(ServoUart2MutexHandle, 5) == osOK) {
              ST_SyncWritePos(&huart4, ids2, n2, pos2, spd2, acc2);
              osMutexRelease(ServoUart2MutexHandle);
          }
      }
  }
}

/* USER CODE BEGIN Header_StartFeedbackTask */
void StartFeedbackTask(void *argument)
{
  CommPacket_t rl_pkt;
  rl_pkt.type = TYPE_RL_STATE;
  
  uint8_t *imu_ptr = rl_pkt.data;
  uint8_t *servo_count_ptr = &rl_pkt.data[36];
  ServoFBParam_t *servo_data_ptr = (ServoFBParam_t*)&rl_pkt.data[37];
  
  // 临时缓冲区 (栈上分配)
  uint8_t group1_ids[6]; uint8_t group1_cnt;
  uint8_t group2_ids[6]; uint8_t group2_cnt;
  uint8_t rx_buf1[72]; // 6 * 12
  uint8_t rx_buf2[72];
  
  for(;;)
  {
      osDelay(5); // 200Hz !
      
      IMU_Parse_Loop(); 
      memcpy(imu_ptr, &g_imu_data, 36);
      g_imu_data.updated = 0; 
      
      *servo_count_ptr = active_servo_count;
      
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

      rl_pkt.len = 37 + active_servo_count * sizeof(ServoFBParam_t);
      if (active_servo_count > 0 || g_imu_data.angle[2] != 0.0f) {
           osMessageQueuePut(CommTxQueueHandle, &rl_pkt, 0, 0);
      }
  }
}
/* USER CODE END Header_StartFeedbackTask */
/* USER CODE BEGIN Header_StartCommRxTask */
void StartCommRxTask(void *argument)
{
  uint8_t byte, state = 0, type = 0, len = 0, idx = 0, checksum = 0;
  uint8_t payload[128];

  for(;;)
  {
      // 1. 等待数据到达信号 (无限等待，不消耗CPU)
      //    只要 USB 中断触发，就会设置 0x01，这里立即醒来
      osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

      // 2. 批量处理环形缓冲区里的所有数据
      //    数据可能是一个字节，也可能是一整包(64字节)
      while (USB_RingBuffer_Pop(&byte)) {
          switch(state) {
              case 0: if (byte == PROTOCOL_HEAD_1) { state = 1; checksum = byte; } break;
              case 1: if (byte == PROTOCOL_HEAD_2) { state = 2; checksum += byte; } else state = 0; break;
              case 2: type = byte; checksum += byte; state = 3; break;
              case 3: len = byte; checksum += byte; idx = 0; 
                      if (len > 120) state = 0; else if (len == 0) state = 5; else state = 4; break;
              case 4: payload[idx++] = byte; checksum += byte; if (idx >= len) state = 5; break;
              case 5: if (byte == checksum) {
                          if (type == TYPE_SERVO_CTRL) {
                              ServoBatch_t batch;
                              batch.count = payload[0];
                              if (batch.count <= MAX_SERVO_COUNT) {
                                  memcpy(batch.params, &payload[1], batch.count * sizeof(ServoCtrlParam_t));
                                  osMessageQueuePut(ServoCmdQueueHandle, &batch, 0, 0);
                              }
                          } else if (type == TYPE_TORQUE_CTRL) {
                              uint8_t enable = payload[0];
                              for(int i=0; i < active_servo_count; i++) {
                                  uint8_t id = active_servo_list[i];
                                  UART_HandleTypeDef *huart = (id <= 6) ? &huart2 : &huart4;
                                  osMutexId_t mutex = (id <= 6) ? ServoUart1MutexHandle : ServoUart2MutexHandle;
                                  if (osMutexAcquire(mutex, 10) == osOK) {
                                      ST_SetTorque(huart, id, enable);
                                      osMutexRelease(mutex);
                                  }
                              }
                          } else if (type == TYPE_PING) {
                              CommPacket_t pong;
                              pong.type = TYPE_PING; pong.len = len;
                              memcpy(pong.data, payload, len);
                              // PING 包优先级极高，直接放入队首 (Priority 1)
                              osMessageQueuePut(CommTxQueueHandle, &pong, 1, 0); 
                          }
                      }
                      state = 0; break;
          }
      }
  }
}

/* USER CODE BEGIN Header_StartCommTxTask */
void StartCommTxTask(void *argument)
{
  CommPacket_t pkt;
  uint8_t buf[140]; 
  for(;;)
  {
      // 优先级高的包 (Ping) 会先出队
      if (osMessageQueueGet(CommTxQueueHandle, &pkt, NULL, osWaitForever) == osOK) {
          buf[0] = PROTOCOL_HEAD_1; buf[1] = PROTOCOL_HEAD_2;
          buf[2] = pkt.type; buf[3] = pkt.len;
          memcpy(&buf[4], pkt.data, pkt.len);
          uint8_t sum = 0;
          int i;
          for(i=0; i < (pkt.len + 4); i++) sum += buf[i];
          buf[pkt.len + 4] = sum;
          
          // === 关键优化: 阻塞式发送 ===
          // 只要 USB 是忙的 (USBD_BUSY)，就一直等，直到发出去为止
          // 这保证了数据不丢失，且 Ping 包不会因为忙而被丢弃
          uint8_t ret = USBD_BUSY;
          while (ret == USBD_BUSY) {
              ret = CDC_Transmit_FS(buf, pkt.len + 5);
              if (ret == USBD_BUSY) {
                  osDelay(1); // 让出 CPU 给 USB 中断去处理发送完成事件
              }
          }
      }
  }
}
/* USER CODE BEGIN Application */
/* USER CODE END Application */