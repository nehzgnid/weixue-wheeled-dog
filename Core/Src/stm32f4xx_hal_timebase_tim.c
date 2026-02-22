/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_hal_timebase_tim.c
  * @brief   HAL time base based on the hardware TIM.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef        htim4;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function configures the TIM4 as a time base source.
  *         The time source is configured  to have 1ms time base with a dedicated
  *         Tick interrupt priority.
  * @note   This function is called  automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock, uwAPB1Prescaler = 0U;

  uint32_t              uwPrescalerValue = 0U;
  uint32_t              pFLatency;

  HAL_StatusTypeDef     status;

  /* Enable TIM4 clock */
  __HAL_RCC_TIM4_CLK_ENABLE();

  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* Get APB1 prescaler */
  uwAPB1Prescaler = clkconfig.APB1CLKDivider;
  /* Compute TIM4 clock */
  if (uwAPB1Prescaler == RCC_HCLK_DIV1)
  {
    uwTimclock = HAL_RCC_GetPCLK1Freq();
  }
  else
  {
    uwTimclock = 2UL * HAL_RCC_GetPCLK1Freq();
  }

  /* Compute the prescaler value to have TIM4 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

  /* Initialize TIM4 */
  htim4.Instance = TIM4;

  /* Initialize TIMx peripheral as follow:
   * Period = [(TIM4CLK/1000) - 1]. to have a (1/1000) s time base.
   * Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
   * ClockDivision = 0
   * Counter direction = Up
   */
  htim4.Init.Period = (1000000U / 1000U) - 1U;
  htim4.Init.Prescaler = uwPrescalerValue;
  htim4.Init.ClockDivision = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  status = HAL_TIM_Base_Init(&htim4);
  if (status == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    status = HAL_TIM_Base_Start_IT(&htim4);
    if (status == HAL_OK)
    {
    /* Enable the TIM4 global Interrupt */
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
      /* Configure the SysTick IRQ priority */
      if (TickPriority < (1UL << __NVIC_PRIO_BITS))
      {
        /* Configure the TIM IRQ priority */
        HAL_NVIC_SetPriority(TIM4_IRQn, TickPriority, 0U);
        uwTickPrio = TickPriority;
      }
      else
      {
        status = HAL_ERROR;
      }
    }
  }

 /* Return function status */
  return status;
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM4 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable TIM4 update Interrupt */
  __HAL_TIM_DISABLE_IT(&htim4, TIM_IT_UPDATE);
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM4 update interrupt.
  * @param  None
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Enable TIM4 Update interrupt */
  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
}

// 引入外部电机控制循环 (Motor.c)
extern void Motor_Control_Loop(void);
extern void Process_RingBuffer(void); // 如果需要更频繁的解析
extern volatile uint8_t Motor_Initialized; // 引入初始化标志 (Checked)

// 重写 HAL_TIM_PeriodElapsedCallback 以在中断中执行高频任务
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4) {
    HAL_IncTick();
    
    // 在这里调用电机控制循环 (1ms 一次)
    // 注意：Motor_Control_Loop 必须非常快且非阻塞！
    // 如果 Process_RingBuffer 耗时较长，可能需要谨慎。
    // 但为了 PID 实时性，必须在这里计算 PWM。
    
    static uint8_t motor_tick = 0;
    motor_tick++;
    if (Motor_Initialized && motor_tick >= 10) { // 10ms (100Hz) 控制频率，且确保已初始化
        motor_tick = 0;
        // 如果 Process_RingBuffer 还没执行完，Motor_Control_Loop 用的还是旧数据
        // 但至少 PID 计算会更新（积分项等）
        
        // 尝试在中断里解析一点点数据？风险较大。
        // 最安全的是只在这里跑 PID 计算，数据的解析还是交给 Main Loop。
        // 但如果 Main Loop 被 ST_SyncRead 阻塞 50ms，那 PID 用的就是 50ms 前的数据...
        // 这就是问题的核心：必须让数据更新也变快。
        
        // 激进方案：在中断里查询 DMA 指针并解析最新的一个包
        // Process_RingBuffer 现在已经优化为"只读最新"，应该可以在中断里跑 (耗时<100us?)，
        // 只要不涉及复杂的内存分配或阻塞操作。
        // Motor_Process_Packet 只是简单的 str->float 和赋值。
        
        Process_RingBuffer(); 
        Motor_Control_Loop();
    }
  }
}

