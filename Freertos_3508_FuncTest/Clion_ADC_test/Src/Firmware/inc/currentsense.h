#ifndef __CURRENTSENSE_H
#define __CURRENTSENSE_H

#include "adc.h"


#define currentGain 		40			//If Gain is high then 40V,else is 10
#define SenResistor			0.0075f		//currentSense Resistor value

/**
 * @brief Current Sense handle
 * @param Myadc:this is the Connect to adc handle outside
 * @param RawADC:Raw value of ADC
 * @param PhaseCurrent:the value of actual current
 * @param offset:offset of phase A and phase B
 * @param PhaseCurrent_Past:use for filer,phase A and phase B
 * @note
 */
typedef struct{
	ADC_HandleTypeDef *Myadc;
	uint32_t RawADC[3];
	float PhaseCurrent[2];				// [1] 为ADC1,为A相，[0]为ADC2，B相。
	float PowerMeasure;					// DC Bus measure
	float offset[2];					// [1] 为ADC1,为A相, [0]为ADC2, B相。
	float PhaseCurrent_Past[2];			// use for current filter
}CurrSenStruct;

/**
 * @brief Initatlize the currentsensor
 * @param [hadc] ADC handle
 * @param Sense: interface with outside
 * @note CailADC is used in this func
 */
void CurrentSensor_Init(ADC_HandleTypeDef *hadc,CurrSenStruct *Sense);
void CaliADC(CurrSenStruct *Sense);
void AcquireCurrent(CurrSenStruct *Sense);
float Current_SenseA(CurrSenStruct *Sense);
float Current_SenseB(CurrSenStruct *Sense);
void CurrentSensor_SetZero(CurrSenStruct *Sense);
/**
 * @brief init some value of CurrSenStruct,and get the offset of ADCs'
 * @param [Sense] CurrSenStruct handle
 * @note 
 */
void CaliADC(CurrSenStruct *Sense);
/**
 * @brief acquire the current current value
 * @param [Sense] CurrSenStruct handle
 * @note 
 */
void AcquireCurrent(CurrSenStruct *Sense);
/**
 * @brief get the current value in phaseA
 * @param 
 * @note 
 */
float Current_SenseA(CurrSenStruct *Sense);
/**
 * @brief get the current value in phaseB
 * @param 
 * @note 
 */
float Current_SenseB(CurrSenStruct *Sense);

void CurrentSensor_SetZero(CurrSenStruct *Sense);

#endif
