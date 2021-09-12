/*
			UTF-8 Decoding
*********			Copyright 2020 MeisseZhong		 ***********
FOC Firmware V1
Impelement Function:
	1)Angle Control
	2)Speed Control
	3)Torue Control(iq Control)

Base on Hardware:
	1)MCU:STM32F334C8T6
	2)MOSFET:CSD18540
	3)Driver:DRV8302
	4)Angle Sensor:TAN sensor and MA700
*/


#include "foc.h"
#include "math.h"
#include "Cmath.h"
#include "stdio.h"
#include "usart.h"
#include "hrtim.h"

uint8_t MOTOR_STATE[LENGTH];//state register
///////////////////////////////////////////////////////////////////////////////////////////////////
void FOC_task(void)
{
	//Actual_Speed_Acquire(&foc_handle,0.001f);
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
}

void FOC_GetSensorTask(void)
{
	Actual_Angle_Acquire(&foc_handle);
	AcquireCurrent(&Sense_handle);
	foc_handle.U_Bus = Sense_handle.PowerMeasure*12;                //Gain the Source Voltage
	Actual_Current_Acquire(&foc_handle,&Sense_handle);
	currentLimit(foc_handle.ia,foc_handle.ib,foc_handle.ic,15.0f,foc_handle.state);
	OverDuty_Detect(foc_handle.svpwm->DutyA,foc_handle.svpwm->DutyB,foc_handle.svpwm->DutyC,foc_handle.state);
	//InputUnder_voltage(foc_handle.U_Bus);
	if(Fast_fabs(foc_handle.iq)>=4.0f)
	{
		ERROR_CODE = 1;
	}
	
	//Actual_Speed_Acquire(&foc_handle,0.001f);
}




///////////////////////////////////////////////////////////////////////////////////////////////////
//initaliztion function



/**
 * @brief To set inital value to all the param that relevent to the moving
 * @param foc:FOC_Struct Pointer
*/
void FOCStruct_Init(FOC_Struct *foc,SVPWMStruct *svpwm)
{

    foc->ia = 0;
    foc->ib = 0;
    foc->ic = 0;

    foc->id_des = 0;
    foc->iq_des = 0;
    foc->iq = 0;
    foc->id = 0;
		foc->iq_int = 0;
		foc->id_int = 0;
		foc->iq_past = 0; //有用的，用于滤波的
		foc->id_past = 0;
		
    foc->iap = 0;
    foc->ibe = 0;

    foc->Uq_des = 0;
    foc->Ud_des = 0;

    foc->Uap_des = 0;
    foc->Ube_des = 0;

    foc->Ua = 0;
    foc->Ub = 0;
    foc->Uc = 0;

    foc->theta = 0;
    foc->theta_old = 0;
		foc->theta_des = 0;
		foc->theta_int= 0;
		
    foc->ve_des = 0;
    foc->ve_act = 0;
		foc->ve_int = 0;

		foc->theta_offset = Angle_Get();
		foc->theta_diff_his = 0;
		foc->OC = 0;
		foc->iq_past2 = 0;
		foc->id_past2 = 0;
		foc->working_state = None;
		foc->state = Pending;
		foc->svpwm = svpwm;
		SVPWM_SetZero(foc->svpwm);
		
		init_PID(foc);
//		init_State();
}

/**
 * @brief To set the direction state and current state
*/
//void init_State(void)
//{
//	DIR_REG = DIR_OK;
//	CUR_REG = CURRENT_OK;
//}

/**
 * @brief Set device ID and store it in flash
 * @param id:The id to be set to the device
*/
void Write_FOC_ID(uint8_t id)
{
	FLASH_EraseInitTypeDef My_Flash;
	uint32_t PageError = 0;
	HAL_FLASH_Unlock();
	My_Flash.TypeErase = FLASH_TYPEERASE_PAGES;
	My_Flash.PageAddress = Flash_Address;//define in foc.c
	My_Flash.NbPages = 1;
	HAL_FLASHEx_Erase(&My_Flash,&PageError);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,Flash_Address,id);//在写入之前，必须得先进行擦除
	HAL_FLASH_Lock();
}

/**
 * @brief Set initial value to PID controller
 * @param foc:FOC_Struct
*/
void init_PID(FOC_Struct *foc)
{
	foc->I_I = 0.0002f;				// intergral parameter too high will have some tremble
	foc->I_P = 0.001f;
	foc->Ve_I = -0.0002;		//i have not idea why this place is negetive value,but it just solve rotation problem(reverse)
	foc->Ve_P = 0.001;
	foc->A_P = 20.0f*dt_50ms;
	foc->A_I = 1*dt;
	foc->A_D = 10*dt_5ms;
	foc->ve_des = 0.0f;			//max is 6000 rpm
	foc->theta_des = PI;
}

void Init_Drv8302(void)
{
  Shut_down();
//  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
//  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1+HRTIM_OUTPUT_TA2);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TB1);
	HAL_HRTIM_WaveformCountStart_IT(&hhrtim1,HRTIM_TIMERID_TIMER_A);
	HAL_HRTIM_WaveformCountStart_IT(&hhrtim1,HRTIM_TIMERID_TIMER_B);
	
//	HAL_HRTIM_WaveformCountStart_IT(&hhrtim1,HRTIM_TIMERID_TIMER_A);
  Start_Drv8302();
}

void Start_Drv8302(void)
{
  HAL_GPIO_WritePin(M_PWM_GPIO_Port,M_PWM_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENABLE_GPIO_Port,ENABLE_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(DC_CAL_GPIO_Port,DC_CAL_Pin,GPIO_PIN_RESET);
}

void Stop_Drv8302(void)
{
//  HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
//  HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
//  HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
  HAL_GPIO_WritePin(ENABLE_GPIO_Port,ENABLE_Pin,GPIO_PIN_RESET);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//control function
/**
 * @brief foc algorithm, including mode switch
*/
void FOC_Trans(FOC_Struct *foc)
{
  Actual_Speed_Acquire(foc,0.001f);
	if(foc->working_state == Speed)
	{
		tuner_PI(foc->Ve_P,foc->Ve_I,&foc->ve_int,&foc->ve_des,&foc->ve_act,&foc->iq_des,20.0f);
	}else if(foc->working_state == Angle)
	{
		Angle_Control(foc);
	}
	
	foc->iq_des = fmax(fmin(foc->iq_des,I_Max),-I_Max);
  foc->id_des = 0;
		
	tuner_PI(foc->I_P,foc->I_I,&foc->iq_int,&foc->iq_des,&foc->iq,&foc->Uq_des,6);
	tuner_PI(foc->I_P,foc->I_I,&foc->id_int,&foc->id_des,&foc->id,&foc->Ud_des,6);
	
	limit_norm(&foc->Ud_des,&foc->Uq_des,Vdc);
		
	float dtc_d = foc->Ud_des/Vdc;
	float dtc_q = foc->Uq_des/Vdc;
	linearize_dtc(&dtc_d);
	linearize_dtc(&dtc_q);
	foc->Ud_des = dtc_d*Vdc;
	foc->Uq_des = dtc_q*Vdc;
		
	foc->theta_old = foc->theta;
	Actual_Angle_Acquire(foc);
	abc(foc->theta/*+foc->ve_act*dt_50ms*/,foc->Ud_des,foc->Uq_des,&foc->Ua,&foc->Ub,&foc->Uc);
	SPWM(foc->svpwm,foc->Ua,foc->Ub,foc->Uc);
	
	foc->svpwm->CCRA = (RELOAD)*(1.0f-foc->svpwm->DutyA);
	foc->svpwm->CCRB = (RELOAD)*(1.0f-foc->svpwm->DutyB);
	foc->svpwm->CCRC = (RELOAD)*(1.0f-foc->svpwm->DutyC);
	
	Set_HrtimDuty(foc->svpwm->CCRA,foc->svpwm->CCRB,foc->svpwm->CCRC);
	if(!foc->OC)
	{
//		TIM1->CCR1 = (RELOAD)*(1.0f-foc->svpwm->DutyC);                        // Write duty cycles
//		TIM1->CCR2 = (RELOAD)*(1.0f-foc->svpwm->DutyB);
//		TIM1->CCR3 = (RELOAD)*(1.0f-foc->svpwm->DutyA);
	}
}

void Set_HrtimDuty(int CHA,int CHB,int CHC)
{
	//Origin CH1:,DutyC
	hhrtim1.Instance->sTimerxRegs[0].CMP1xR = HALF_RELOAD-CHC/2;
	hhrtim1.Instance->sTimerxRegs[0].CMP2xR = HALF_RELOAD+CHC/2;
	//Origin CH2:,DutyB
	hhrtim1.Instance->sTimerxRegs[0].CMP3xR = HALF_RELOAD-CHB/2;
	hhrtim1.Instance->sTimerxRegs[0].CMP4xR = HALF_RELOAD+CHB/2;
	//Origin CH3:,DutyA
	hhrtim1.Instance->sTimerxRegs[1].CMP1xR = HALF_RELOAD-CHA/2;
	hhrtim1.Instance->sTimerxRegs[1].CMP2xR = HALF_RELOAD+CHA/2;
}
void Angle_Control(FOC_Struct *foc)
{
	//It can be PID tuner
	float A_diff;
	A_diff = foc->theta-foc->theta_des;
	if(A_diff>PI)
	{
		A_diff -= PI2;
	}
	tuner_PID(foc->A_P,foc->A_I,foc->A_D,&foc->theta_int,&foc->theta_diff_his,A_diff,&foc->iq_des,A_Max,dt);
}

/**
 * @brief: PI handle
 * @param Para_P: properity parameter
 * @param Para_I: intergal parameter
 * @param intergal_val: as its name
 * @param Desti_data: The destination data that I want to get by using the PI
 * @param Current_data: The data in the feed back loop now
 * @param New_Desti_data: Output value of the PI handle
 * @param limit: intergal limitation, in case of saturation
 * */
void tuner_PI(float Para_P,float Para_I,float *intergal_val,float *Desti_data,float *Current_data,float *New_Desti_data,float limit)
{
	float dif;
	dif = *Desti_data-*Current_data;
	*intergal_val += dif*Para_I;
	*intergal_val = fmax(fmin(*intergal_val,limit),-limit);
	*New_Desti_data = dif*Para_P + *intergal_val;
}
/**
 * @brief : similar to PI handle
 * @param DT:Time gap between using the PID handle
 * @param Para_D: differentate parameter. Using for the diff_his and dif
 * */
void tuner_PID(float Para_P,float Para_I,float Para_D,float *diff_his,float *intergal_val,float dif,float *New_Desti_data,float limit,float DT)
{
	float temp;
	*intergal_val += dif*Para_I;
	*intergal_val = fmax(fmin(*intergal_val,limit),-limit);
	temp = Para_D*(dif-*diff_his)/DT;
	temp = fmax(fmin(temp,limit),-limit);
	*New_Desti_data = dif*Para_P + *intergal_val+temp;
	*diff_his = dif;
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//data process
/**
 * @brief: get the actual current.And excute the transformation, also fillter the result.This process is solid
 * @param foc: FOC_Sturct,to have the data communication
 * @param Sense: CurrSenSturct,to get current from ADC
 * */
void Actual_Current_Acquire(FOC_Struct *foc,CurrSenStruct *Sense)
{
  foc->ia = Current_SenseA(Sense);
  foc->ib = Current_SenseB(Sense);

  foc->ic = -foc->ia-foc->ib;
	dq0(foc->theta,foc->ia,foc->ib,foc->ic,&foc->id,&foc->iq);
	LowPassFilter_RC_1order(&foc->iq,&foc->iq,&foc->iq_past,1/dt,0.25f);
	LowPassFilter_RC_1order(&foc->id,&foc->id,&foc->id_past,1/dt,0.25f);
}

//plan to put it in high task
/**
 * @brief: To get the actual angle
 * */
void Actual_Angle_Acquire(FOC_Struct *foc)
{
	foc->theta = Angle_Get();/*-foc->theta_offset*/;//float calculation and compare with 0 is improper
	if(foc->theta<0)
  {
    foc->theta+=PI2;
  }
}
//plan to put it in mid task

/**
 * @brief :To get the Actual Speed
 * */
void Actual_Speed_Acquire(FOC_Struct *foc,float DT)
{
	float dtheta = foc->theta-foc->theta_old;

	if(dtheta<-3.0f)
	{
		foc->ve_act = ((dtheta + PI2)/DT);
	}
	else if(dtheta > 3.0f)
	{
		foc->ve_act = ((dtheta - PI2)/DT);
	}
	else
	{
		foc->ve_act = (dtheta/DT);
	}
#ifdef TAN_positionSenser
	foc->ve_act /= NPP;
#endif
  foc->theta_old = foc->theta;
}

/**
 * @brief :
 * */
void linearize_dtc(float *dtc){
    /// linearizes the output of the inverter, which is not linear for small duty cycles ///
    float sgn = 1.0f-(2.0f*(dtc<0));
    if(fabs(*dtc) >= .01f){
        *dtc = *dtc*.986f+.014f*sgn;
        }
    else{
        *dtc = 2.5f*(*dtc);
        }
    }


/////////////////////////////////////////////////////////////////////////////////////////////////
//test function
/**
 * @brief : Only use in debug,just to test the current in the CurrentSense Resistor.And the power supply must have
 * current limit
 * */
//void Current_Test_Fuc(void)
//{
//	TIM1->CCR3 = RELOAD*0.01;
//	TIM1->CCR1 = RELOAD;
//	TIM1->CCR2 = RELOAD;
//}


/////////////////////////////////////////////////////////////////////////////////////////////////
//safe function

void currentLimit(float ia,float ib,float ic,float threshold,int state)
{
  if(state == Moving)
  {
    if(Fast_fabs(ia) >threshold || Fast_fabs(ib) >threshold || Fast_fabs(ic)>threshold)
    {
      Shut_down();
      Stop_Drv8302();
      ERROR_CODE = 1;
//      DMA_printf("ERRORcode=%d\r\n",ERROR_CODE);
    }
  }
}

void OverDuty_Detect(float dutyA,float dutyB,float dutyC,int state)
{
  if(state == Moving) {
    if (dutyA >= MAXDutyPersent || dutyB >= MAXDutyPersent || dutyC >= MAXDutyPersent) {
      Shut_down();
      Stop_Drv8302();
      ERROR_CODE = 2;
//      DMA_printf("ERRORcode=%d\r\n",ERROR_CODE);
    }
  }
}
void InputUnder_voltage(float Voltage)
{
  if(Voltage<=V_Min)
  {
    Shut_down();
    Stop_Drv8302();
    ERROR_CODE=3;
//    DMA_printf("ERRORcode=%d\r\n",ERROR_CODE);
  }
}
/**
 * @brief Set all the channel of TIM1 to zero to stop the motor.And stop the Drv8302 and Reset the State
 * */
void Shut_down(void)
{
	Set_HrtimDuty(0,0,0);
  foc_handle.state = Pending;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
