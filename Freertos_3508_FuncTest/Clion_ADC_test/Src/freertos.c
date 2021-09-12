/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "foc.h"
#include "usart.h"
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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId FOC_LOGICHandle;
osThreadId Safe_CheckHandle;
osThreadId Send_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void FOC_LOGICFuc(void const * argument);
void Safe_CheckFuc(void const * argument);
void Send_TaskFuc(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of FOC_LOGIC */
  osThreadDef(FOC_LOGIC, FOC_LOGICFuc, osPriorityNormal, 0, 256);
  FOC_LOGICHandle = osThreadCreate(osThread(FOC_LOGIC), NULL);

  /* definition and creation of Safe_Check */
  osThreadDef(Safe_Check, Safe_CheckFuc, osPriorityNormal, 0, 128);
  Safe_CheckHandle = osThreadCreate(osThread(Safe_Check), NULL);

  /* definition and creation of Send_Task */
  osThreadDef(Send_Task, Send_TaskFuc, osPriorityNormal, 0, 128);
  Send_TaskHandle = osThreadCreate(osThread(Send_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	//Perip_Initalization();
  for(;;)
  {
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_FOC_LOGICFuc */
/**
* @brief Function implementing the FOC_LOGIC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FOC_LOGICFuc */
void FOC_LOGICFuc(void const * argument)
{
  /* USER CODE BEGIN FOC_LOGICFuc */
  /* Infinite loop */
//	Perip_Initalization();
  for(;;)
  {
		FOC_GetSensorTask();
		FOC_task();
    osDelay(1);
		HAL_ADCEx_Calibration_Start(Sense_handle.Myadc,ADC_SINGLE_ENDED);
		HAL_ADC_Start_DMA(Sense_handle.Myadc, Sense_handle.RawADC, 3);
		
  }
  /* USER CODE END FOC_LOGICFuc */
}

/* USER CODE BEGIN Header_Safe_CheckFuc */
/**
* @brief Function implementing the Safe_Check thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Safe_CheckFuc */
void Safe_CheckFuc(void const * argument)
{
  /* USER CODE BEGIN Safe_CheckFuc */
  /* Infinite loop */
  for(;;)
  {
		//FOC_GetSensorTask();
		//DMA_printf("ia=%f\r\n",foc_handle.ia);
    osDelay(1);
  }
  /* USER CODE END Safe_CheckFuc */
}

/* USER CODE BEGIN Header_Send_TaskFuc */
/**
* @brief Function implementing the Send_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Send_TaskFuc */
void Send_TaskFuc(void const * argument)
{
  /* USER CODE BEGIN Send_TaskFuc */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Send_TaskFuc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
