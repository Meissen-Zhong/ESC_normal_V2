#include "midtask.h"
#include "main.h"
#include "foc.h"
#include "stdio.h"
#include "usart.h"
//midtask is operated in the TIM6 interrupt,priod of TIM6 is 0.0005s

int TaskCounter_500ms=0;
int TaskCounter_50ms=0;
int TaskCounter_5ms=0;
int TaskCounter_00_5ms = 0;

void Task_500ms(void)
{
	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
}

void Task_50ms(void)
{
//  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
//  DMA_printf("50ms\n");
}

void Task_5ms(void)
{
//  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
//  DMA_printf("5ms\n");
}

void Task_00_5ms(void)
{
//  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
//  DMA_printf("005ms\n");
}

void Central_Manager(void)
{
	if(TaskCounter_500ms>=10000)
	{
		Task_500ms();
		TaskCounter_500ms = 0;
	}
	if(TaskCounter_50ms >= 1000)
	{
		Task_50ms();
		TaskCounter_50ms = 0;
	}
	if(TaskCounter_5ms>=100)
	{
		Task_5ms();
		TaskCounter_5ms = 0;
	}
	if(TaskCounter_00_5ms>=1)
	{
		Task_00_5ms();
		TaskCounter_00_5ms = 0;
	}
}

void HAL_TIM_PeriodElapsedCallback_ThrowOff(TIM_HandleTypeDef *htim) {
  if(htim->Instance == TIM1)
  {
    Actual_Angle_Acquire(&foc_handle);
    AcquireCurrent(&Sense_handle);
    foc_handle.U_Bus = Sense_handle.PowerMeasure*18;                //Gain the Source Voltage
    Actual_Current_Acquire(&foc_handle,&Sense_handle);
    currentLimit(foc_handle.ia,foc_handle.ib,foc_handle.ic,15.0f,foc_handle.state);
    OverDuty_Detect(foc_handle.svpwm->DutyA,foc_handle.svpwm->DutyB,foc_handle.svpwm->DutyC,foc_handle.state);
    InputUnder_voltage(foc_handle.U_Bus);
    if(ERROR_CODE)
    {
      foc_handle.state = Pending;
    }
    switch(foc_handle.state)
    {
      case Pending:
        Shut_down();
        //將控制值進行初始化，避免耦合进下一次操作
        SVPWM_SetZero(foc_handle.svpwm);
        foc_handle.iq_int = 0;
        foc_handle.id_int = 0;
        foc_handle.ve_int = 0;
        foc_handle.theta_int = 0;
        break;
      case Moving:
        FOC_Trans(&foc_handle);
        break;
      case CurrentTest:
        Shut_down();
        break;
      default:
        Shut_down();
        break;
    }
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
  }

  if(htim->Instance == TIM6)
  {
    TaskCounter_500ms ++;
    TaskCounter_50ms ++;
    TaskCounter_5ms ++;
    TaskCounter_00_5ms ++;
    Central_Manager();
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
  }
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
}

