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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 舵机批处理指令结构 (用于任务间传递)
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
// === 1. 全局句柄定义 (这里分配内存) ===
osThreadId_t ServoTaskHandle;
osThreadId_t FeedbackTaskHandle;
osThreadId_t CommRxTaskHandle;
osThreadId_t CommTxTaskHandle;

osMessageQueueId_t CommTxQueueHandle;
osMessageQueueId_t ServoCmdQueueHandle;
osMutexId_t ServoUartMutexHandle;

// === 2. 任务属性定义 ===
const osThreadAttr_t ServoTask_attributes = {
  .name = "ServoTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

const osThreadAttr_t FeedbackTask_attributes = {
  .name = "FeedbackTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

const osThreadAttr_t CommRxTask_attributes = {
  .name = "CommRxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t CommTxTask_attributes = {
  .name = "CommTxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

// === 3. 本地变量 ===
static uint8_t active_servo_list[] = {1, 2, 3, 4, 5, 6}; 
static uint8_t active_servo_count = 6;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartServoTask(void *argument);
void StartFeedbackTask(void *argument);
void StartCommRxTask(void *argument);
void StartCommTxTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); 
/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  const osMutexAttr_t ServoUartMutex_attributes = { .name = "ServoUartMutex" };
  ServoUartMutexHandle = osMutexNew(&ServoUartMutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_QUEUES */
  // 发送队列: 8个元素, 每个128字节
  CommTxQueueHandle = osMessageQueueNew(8, 128, NULL);
  // 指令队列: 4个元素, 每个128字节
  ServoCmdQueueHandle = osMessageQueueNew(4, 128, NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  ServoTaskHandle = osThreadNew(StartServoTask, NULL, &ServoTask_attributes);
  FeedbackTaskHandle = osThreadNew(StartFeedbackTask, NULL, &FeedbackTask_attributes);
  CommRxTaskHandle = osThreadNew(StartCommRxTask, NULL, &CommRxTask_attributes);
  CommTxTaskHandle = osThreadNew(StartCommTxTask, NULL, &CommTxTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartCommRxTask */
/**
  * @brief  CommRxTask: 解析 USB 数据管道
  */
/* USER CODE END Header_StartCommRxTask */
void StartCommRxTask(void *argument)
{
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
                            // --- 分发逻辑 ---
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
}

/* USER CODE BEGIN Header_StartCommTxTask */
/**
  * @brief  CommTxTask: 统一 USB 输出，确保发送不冲突
  */
/* USER CODE END Header_StartCommTxTask */
void StartCommTxTask(void *argument)
{
    CommPacket_t pkt;
    uint8_t buf[132]; // Head(2)+Type(1)+Len(1)+Payload(120)+Check(1)
    
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
}

/* USER CODE BEGIN Header_StartServoTask */
/**
  * @brief  ServoTask: 高优先级舵机控制
  */
/* USER CODE END Header_StartServoTask */
void StartServoTask(void *argument)
{
  osDelay(100);       // 等待电源稳定
  MX_USB_DEVICE_Init(); // 必须在此初始化 USB，因为 main.c 中未调用
  
  ServoBatch_t cmd;
  // 临时数组用于底层 SyncWrite 调用
  uint8_t ids[MAX_SERVO_COUNT];
  int16_t positions[MAX_SERVO_COUNT];
  uint16_t speeds[MAX_SERVO_COUNT];
  uint8_t accs[MAX_SERVO_COUNT];

  osDelay(500); // 等待上电稳定

  // 初始上锁
  if (osMutexAcquire(ServoUartMutexHandle, osWaitForever) == osOK) {
    int i;
    for(i=0; i<6; i++) { // 默认给前6个上锁
        ST_SetTorque(active_servo_list[i], 1);
        HAL_Delay(5);
    }
    osMutexRelease(ServoUartMutexHandle);
  }

  for(;;)
  {
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
}

/* USER CODE BEGIN Header_StartFeedbackTask */
/**
  * @brief  FeedbackTask: 状态轮询与上报
  */
/* USER CODE END Header_StartFeedbackTask */
void StartFeedbackTask(void *argument)
{
  CommPacket_t fb_pkt;
  ServoFBParam_t *fb_data = (ServoFBParam_t*)&fb_pkt.data[1]; // data[0] 留给 Count
  
  for(;;)
  {
      osDelay(10); // 100Hz
      
      fb_pkt.type = TYPE_SERVO_FB;
      fb_pkt.data[0] = active_servo_count; // 第一字节是舵机数量
      
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
}

/* USER CODE BEGIN Application */
/* USER CODE END Application */
