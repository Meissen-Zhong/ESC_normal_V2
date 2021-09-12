#include "currentsense.h"
#include "stdio.h"
#include "Cmath.h"
#include "foc.h"

//only a CurrentStruct to commuicate with outside.
CurrSenStruct *Sense_pointer;

void CurrentSensor_Init(ADC_HandleTypeDef *hadc,CurrSenStruct *Sense)
{
  Sense_pointer = Sense;
	Sense_pointer->Myadc = hadc;
	CaliADC(Sense_pointer);
}

void CaliADC(CurrSenStruct *Sense)
{
	Sense->offset[0] = 0;
	Sense->offset[1] = 0;
	Sense->PhaseCurrent[0] = 0;
	Sense->PhaseCurrent[1] = 0;
	Sense->PowerMeasure = 0;
	Sense->PhaseCurrent_Past[0] = 0;
	Sense->PhaseCurrent_Past[1] = 0;
//	HAL_ADC_Start_IT(Sense->Myadc);
	HAL_ADCEx_Calibration_Start(Sense->Myadc,ADC_SINGLE_ENDED);
//	HAL_ADCEx_Calibration_Start(Sense->Myadc,ADC_SINGLE_ENDED);
	AcquireCurrent(Sense);
	
	Sense->offset[0] = 1.65f;		//offset[0]ΪB��ģ�
	Sense->offset[1] = 1.65f;		//offset[1]ΪA��ģ�
}

void CurrentSensor_SetZero(CurrSenStruct *Sense)
{
	Sense->PhaseCurrent[0] = 0;
	Sense->PhaseCurrent[1] = 0;
	Sense->PhaseCurrent_Past[0] = 0;
	Sense->PhaseCurrent_Past[0] = 0;
}

void AcquireCurrent(CurrSenStruct *Sense)
{
//	Sense->RawADC[0] = 0;
//	Sense->RawADC[1] = 0;
//	Sense->RawADC[2] = 0;
	//��Ϊ������ͨ����ÿ��ͨ���ɼ����֮�󣬾ͽ���һ���жϣ�����ͨ���ɼ���Ͼͽ��������жϣ������Ҫ��������������
//	for(int temp_k2 = 0;temp_k2<3;temp_k2++){
//    //HAL_ADCEx_Calibration_Start(Sense->Myadc,ADC_SINGLE_ENDED);
//		for(int temp_k3 = 0;temp_k3 <10;temp_k3++);
//  HAL_ADC_Start_DMA(Sense->Myadc, Sense->RawADC, 3);
//  }

	Sense->PowerMeasure = (float)Sense->RawADC[0]*3.3f/4096;	
	Sense->PhaseCurrent[0] = (float)Sense->RawADC[1]*3.3f/4096;	//currentGain/SenResistor;	//12λADC�������ֵ,1��ӦChannel_2,ΪB��
	Sense->PhaseCurrent[1] = (float)Sense->RawADC[2]*3.3f/4096;	//currentGain/SenResistor;	//��ӦChannel_1��ΪA��
}


float Current_SenseA(CurrSenStruct *Sense)
{
	float currentA;
	currentA = (Sense->offset[1]-Sense->PhaseCurrent[1])/currentGain/SenResistor;
//	Matlab_FIR(&currentA,&currentA,1);
	return currentA;
}

float Current_SenseB(CurrSenStruct *Sense)
{
	float currentB;
	currentB = (Sense->offset[0]-Sense->PhaseCurrent[0])/currentGain/SenResistor;
//	Matlab_FIR(&currentB,&currentB,1);
	return currentB;
}



