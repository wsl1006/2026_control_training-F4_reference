/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Arm_StateMachine.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* Definitions for ArmTask(机械臂状态机任务) */
osThreadId_t ArmTaskHandle;
const osThreadAttr_t ArmTask_attributes = {
  .name = "ArmTask",
  /* 512 字(word) 的栈 = 2048 字节 */
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE END Variables */
/* Definitions for LedTask */
osThreadId_t LedTaskHandle;
const osThreadAttr_t LedTask_attributes = {
  .name = "LedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BeepTask */
osThreadId_t BeepTaskHandle;
const osThreadAttr_t BeepTask_attributes = {
  .name = "BeepTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LedWaterTask(void *argument);
void BeepAlarmTask(void *argument);
void ArmTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /*
   * 创建机械臂命令队列。
   *
   * 任务本体在下面 RTOS_THREADS 里创建。
   */
  Arm_StateMachine_Init();

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LedTask */
  LedTaskHandle = osThreadNew(LedWaterTask, NULL, &LedTask_attributes);

  /* creation of BeepTask */
  BeepTaskHandle = osThreadNew(BeepAlarmTask, NULL, &BeepTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /*
   * 创建机械臂状态机任务。
   *
   * 任务体就是下面定义的这个 ArmTask()，
   * 内容委托给 Motor/Src/Arm_StateMachine.c 的 Arm_Task()。
   *
   * 上电安全：
   * 四电机初始都 Disable / 0 电流，
   * 只有收到 CAN 动作命令(StdId 0x01..0x08)才会运动。
   *
   * 需要调优先级/栈时改上面的 ArmTask_attributes。
   */
  ArmTaskHandle = osThreadNew(ArmTask, NULL, &ArmTask_attributes);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_LedWaterTask */
/**
  * @brief  Function implementing the LedTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LedWaterTask */
__weak void LedWaterTask(void *argument)
{
  /* USER CODE BEGIN LedWaterTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LedWaterTask */
}

/* USER CODE BEGIN Header_BeepAlarmTask */
/**
* @brief Function implementing the BeepTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BeepAlarmTask */
__weak void BeepAlarmTask(void *argument)
{
  /* USER CODE BEGIN BeepAlarmTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END BeepAlarmTask */
}

/* USER CODE BEGIN Header_ArmTask */
/**
* @brief Function implementing the ArmTask thread.
*
* 机械臂状态机任务。
*
* 逻辑在 Motor/Src/Arm_StateMachine.c 的 Arm_Task() 里：
* 命令队列 -> 左右机械臂状态机 -> 电机位置模式 -> 气泵。
*
* 安全：上电电机始终 Disable / 0 电流，
* 只有收到 CAN 动作命令(StdId 0x01..0x08)才会运动。
*
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ArmTask */
__weak void ArmTask(void *argument)
{
  /* USER CODE BEGIN ArmTask */
  Arm_Task(argument);
  /* USER CODE END ArmTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

