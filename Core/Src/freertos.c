/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
// 舵机批处理指令结构
typedef struct {
    uint8_t count;
    ServoCtrlParam_t params[MAX_SERVO_COUNT];
} ServoBatch_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FEEDBACK_PERIOD_MS 10 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static uint8_t active_servo_list[] = {1, 2, 3, 4, 5, 6}; 
static uint8_t active_servo_count = 6;
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
/* Definitions for ServoUartMutex */
osMutexId_t ServoUartMutexHandle;
const osMutexAttr_t ServoUartMutex_attributes = {
  .name = "ServoUartMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void MX_USB_DEVICE_Init(void);
/* USER CODE END FunctionPrototypes */

void StartServoTask(void *argument);
void StartFeedbackTask(void *argument);
void StartCommRxTask(void *argument);
void StartCommTxTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  IMU_Init(); // 启动 IMU DMA
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of ServoUartMutex */
  ServoUartMutexHandle = osMutexNew(&ServoUartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CommTxQueue */
  CommTxQueueHandle = osMessageQueueNew (8, 128, &CommTxQueue_attributes);

  /* creation of ServoCmdQueue */
  ServoCmdQueueHandle = osMessageQueueNew (4, 128, &ServoCmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
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
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
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
  // MX_USB_DEVICE_Init(); // 这里的自动生成代码有时会引发问题，我们移到下面手动调用
  /* USER CODE BEGIN StartServoTask */
  
  osDelay(100);       // 等待电源稳定
  MX_USB_DEVICE_Init(); // 手动初始化 USB (确保 main.c 没调用过)
  
  ServoBatch_t cmd;
  // 临时数组
  uint8_t ids[MAX_SERVO_COUNT];
  int16_t positions[MAX_SERVO_COUNT];
  uint16_t speeds[MAX_SERVO_COUNT];
  uint8_t accs[MAX_SERVO_COUNT];

  osDelay(500); 

  // 初始上锁
  if (osMutexAcquire(ServoUartMutexHandle, osWaitForever) == osOK) {
    int i;
    for(i=0; i<6; i++) { 
        ST_SetTorque(active_servo_list[i], 1);
        HAL_Delay(5);
    }
    osMutexRelease(ServoUartMutexHandle);
  }

  /* Infinite loop */
  for(;;)
  {
      // 等待指令
      if (osMessageQueueGet(ServoCmdQueueHandle, &cmd, NULL, osWaitForever) == osOK) {
          if (osMutexAcquire(ServoUartMutexHandle, 10) == osOK) {
              int i;
              for(i=0; i<cmd.count; i++) {
                  ids[i] = cmd.params[i].id;
                  positions[i] = cmd.params[i].pos;
                  speeds[i] = cmd.params[i].speed;
                  accs[i] = cmd.params[i].acc;
              }
              ST_SyncWritePos(ids, cmd.count, positions, speeds, accs);
              osMutexRelease(ServoUartMutexHandle);
          }
      }
  }
  /* USER CODE END StartServoTask */
}

/* USER CODE BEGIN Header_StartFeedbackTask */
/**
* @brief Function implementing the FeedbackTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFeedbackTask */
void StartFeedbackTask(void *argument)
{
  /* USER CODE BEGIN StartFeedbackTask */
  CommPacket_t fb_pkt;
  CommPacket_t imu_pkt;
  
  ServoFBParam_t *fb_data = (ServoFBParam_t*)&fb_pkt.data[1]; 
  
  for(;;)
  {
      osDelay(10); // 100Hz
      
      // 1. IMU 处理
      IMU_Parse_Loop();
      
      if (g_imu_data.updated) {
          g_imu_data.updated = 0;
          imu_pkt.type = TYPE_SENSOR_IMU;
          imu_pkt.len = sizeof(IMU_Data_t) - 1; 
          memcpy(imu_pkt.data, &g_imu_data, imu_pkt.len);
          osMessageQueuePut(CommTxQueueHandle, &imu_pkt, 0, 0);
      }
      
      // 2. 舵机读取
      fb_pkt.type = TYPE_SERVO_FB;
      fb_pkt.data[0] = active_servo_count; 
      
      int i;
      for(i=0; i < active_servo_count; i++) {
          uint8_t id = active_servo_list[i];
          if (osMutexAcquire(ServoUartMutexHandle, 5) == osOK) {
              ST_ReadInfo(id, &fb_data[i].pos, &fb_data[i].speed, &fb_data[i].load);
              fb_data[i].id = id;
              osMutexRelease(ServoUartMutexHandle);
          }
          osDelay(1); 
      }
      fb_pkt.len = 1 + active_servo_count * sizeof(ServoFBParam_t);
      osMessageQueuePut(CommTxQueueHandle, &fb_pkt, 0, 0);
  }
  /* USER CODE END StartFeedbackTask */
}

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
  uint8_t byte, state = 0, type = 0, len = 0, idx = 0, checksum = 0;
  uint8_t payload[128];

  for(;;)
  {
      if (USB_RingBuffer_Pop(&byte)) {
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
                          } else if (type == TYPE_PING) {
                              CommPacket_t pong;
                              pong.type = TYPE_PING;
                              pong.len = len;
                              memcpy(pong.data, payload, len);
                              osMessageQueuePut(CommTxQueueHandle, &pong, 0, 0);
                          }
                      }
                      state = 0; break;
          }
      } else { osDelay(1); }
  }
  /* USER CODE END StartCommRxTask */
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
  uint8_t buf[132]; 
  
  for(;;)
  {
      if (osMessageQueueGet(CommTxQueueHandle, &pkt, NULL, osWaitForever) == osOK) {
          buf[0] = PROTOCOL_HEAD_1;
          buf[1] = PROTOCOL_HEAD_2;
          buf[2] = pkt.type;
          buf[3] = pkt.len;
          memcpy(&buf[4], pkt.data, pkt.len);
          
          uint8_t sum = 0;
          int i;
          for(i=0; i < (pkt.len + 4); i++) sum += buf[i];
          buf[pkt.len + 4] = sum;
          
          CDC_Transmit_FS(buf, pkt.len + 5);
      }
  }
  /* USER CODE END StartCommTxTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */