#ifndef __POSITIONSENSOR_H
#define __POSITIONSENSOR_H

#include "Cmath.h"
#include "lut.h"

#define M		8192    //Accuracy of MA700
#define MA700_CS_Port   GPIOA
#define MA700_CS_Pin    GPIO_PIN_4
#define MA700_SPI       hspi1

/**
 * @brief :pick which senser to use
 * */
//#define MA700_positionSenser
#define TAN_positionSenser

//#define LeveL1_Filter
//#define Avarege_Filter
#define Weight_Filter

#define SIZE 	512
#define PI_4	(PI/4)
#define PI_2 	(PI/2)

float Angle_Get(void);
void TestAngleADC(uint32_t *Val);
void TransRece(uint8_t *write,uint8_t *read);
void GetOffSet(int *Max,int *Min);
#endif
