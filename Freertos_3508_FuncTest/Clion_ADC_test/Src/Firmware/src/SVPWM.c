#include "SVPWM.h"
#include "Cmath.h"
#include "foc.h"
#include "stdio.h"
#include "positionsensor.h"
// orgin name is InitSVPWM
/**
 * @brief 将设置的值进行归零操作
*/
void SVPWM_SetZero(SVPWMStruct *SVPWM)
{
    SVPWM->DutyA = 0;
    SVPWM->DutyB = 0;
    SVPWM->DutyC = 0;
    SVPWM->CCRA = 0;
    SVPWM->CCRB = 0;
    SVPWM->CCRC = 0;
}
/**
 * @brief 三相坐标与两相转子坐标系进行转换
*/
void dq0(float theta, float a, float b, float c, float *d, float *q)
	{
    /// DQ0 Transform ///
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///
#ifdef MA700_positionSenser
    theta *= NPP;
#endif
    float cf = CosTable(theta);
    float sf = SinTable(theta);
    *d = 0.6666667f*(cf*a + (0.86602540378f*sf-.5f*cf)*b + (-0.86602540378f*sf-.5f*cf)*c);   ///Faster DQ0 Transform
    *q = 0.6666667f*(-sf*a - (-0.86602540378f*cf-.5f*sf)*b - (0.86602540378f*cf-.5f*sf)*c);
    }

/**
 * @brief 两相转子坐标转换为三相坐标
*/
void abc( float theta, float d, float q, float *a, float *b, float *c)
{
    /// Inverse DQ0 Transform ///
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///
#ifdef MA700_positionSenser
		theta *= NPP;
#endif
    float cf = CosTable(theta);
    float sf = SinTable(theta);
    *a = cf*d - sf*q;                // Faster Inverse DQ0 transform
    *b = (0.86602540378f*sf-.5f*cf)*d - (-0.86602540378f*cf-.5f*sf)*q;
    *c = (-0.86602540378f*sf-.5f*cf)*d - (0.86602540378f*cf-.5f*sf)*q;
    }


/**
 * @brief 进行占空比的限制，防止过小或者过高
*/
void SPWM(SVPWMStruct *S_IN,float a,float b,float c)
{
	
    float v_offset = (fminf3(a, b, c) + fmaxf3(a, b, c))*0.5f;
  
    S_IN->DutyA = fminf(fmaxf(((a -v_offset)/Vdc + .5f), 0.0f), 0.94f);
    S_IN->DutyB = fminf(fmaxf(((b -v_offset)/Vdc + .5f), 0.0f), 0.94f);
    S_IN->DutyC = fminf(fmaxf(((c -v_offset)/Vdc + .5f), 0.0f), 0.94f); 
    
}
