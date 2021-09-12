#ifndef __CMATH_H
#define __CMATH_H

#include "main.h"

#define Lut_Num 512									
#define PI 3.14159265358979f
#define PI2 (PI*2)							
#define PI23 (PI2/3)						
#define Fast_fabs(a)  a*(1-2*(a<0))
#define FRAME_READ_LENGTH   (1)
#define FIR_FILTER_LENGTH 5

//const float FIR_COFFES[FIR_FILTER_LENGTH] = {
//    0.00265733166144,  0.01501600845409,   0.0184858224731,-0.004377468616168,
//  -0.007149146761303, 0.009738306643678,-0.005017842189598,-0.003413024488775,
//    0.01038313643023, -0.01081081693753, 0.002900172722542, 0.009307836357888,
//   -0.01745856999787,  0.01397081334299,  0.00170867393599, -0.02084114044855,
//    0.02934520449825, -0.01696305522404, -0.01389963883815,  0.04587267982169,
//    -0.0538730004674,  0.01899229473432,  0.05874568314813,  -0.1566561283685,
//     0.2380998209676,   0.7302867079987,   0.2380998209676,  -0.1566561283685,
//    0.05874568314813,  0.01899229473432,  -0.0538730004674,  0.04587267982169,
//   -0.01389963883815, -0.01696305522404,  0.02934520449825, -0.02084114044855,
//    0.00170867393599,  0.01397081334299, -0.01745856999787, 0.009307836357888,
//   0.002900172722542, -0.01081081693753,  0.01038313643023,-0.003413024488775,
//  -0.005017842189598, 0.009738306643678,-0.007149146761303,-0.004377468616168,
//     0.0184858224731,  0.01501600845409,  0.00265733166144
//};

extern float state[FIR_FILTER_LENGTH+1];
/**
 * @brief   Sin function look up table
 * @param   theta:angle value in radius
 * @note
 */
float SinTable(float theta);

/**
 * @brief   Cos function look up table
 * @param   theta:angle value in radius
 * @note
 */
float CosTable(float theta);

/**
 * @brief   find the minium value among the three
 * @param   [x,y,z] is the three compare value
 * @note
 */
float fminf3(float x, float y, float z);

/**
 * @brief find the maximum value in two
 * @param
 * @note
 */
float fmaxf(float x, float y);

/**
 * @brief   find the minimum value in two
 * @param
 * @note
 */
float fminf(float x, float y);

/**
 * @brief   find the maximum value among three
 * @param
 * @note
 */
float fmaxf3(float x, float y, float z);

/**
 * @brief   one level digital lowpast filter
 * @param   vi:input value
 * @param   Vo:value after filting
 * @param   Vo_p:past value of output,just for update
 * @param   sampleFrq:Sample Frquency of sampler
 * @param   CutFrq:stop freqnency
 * @note
 */
//void LowPassFilter_RC_1order(float *Vi, float *Vo, float *Vo_p, float *Vi2,double sampleFrq ,float CutFrq);
// void Matlab_FIR(float *x,float *y,int Num);
// float SinTable(float theta);
// float CosTable(float theta);
void LowPassFilter_RC_1order(float *Vi, float *Vo, float *Vo_p, double sampleFrq ,float CutFrq);
// uint8_t Get_Crc8(uint8_t *ptr,uint16_t len);
// double atan2LUT(double y,double x);
/**
 * @brief   Scales the lenght of vector (x, y) to be <= limit
 * @param   x:x value
 * @param   y:y value
 * @param   limit:the maximum value of the sum of x and y
 * @note
 */
void limit_norm(float *x, float *y, float limit);

/**
 * @brief   in words
 * @param
 * @note
 */
float uint8_to_float(uint8_t *BUFFER);

float uint_to_float(int x_int, float x_min, float x_max, int bits);

void Matlab_FIR(float *x,float *y,int Num);

int float_to_uint(float x, float x_min, float x_max, int bits);

uint8_t Get_Crc8(uint8_t *ptr,uint16_t len);

#endif
