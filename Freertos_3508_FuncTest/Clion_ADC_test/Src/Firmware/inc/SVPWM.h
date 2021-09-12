#ifndef __SVPWM_H
#define __SVPWM_H

#define Vdc     V_Max
#define sqrt3   1.73205081f

typedef struct
{
    float DutyA,DutyB,DutyC;
    int CCRA,CCRB,CCRC;
}SVPWMStruct;

void SVPWM_SetZero(SVPWMStruct *SVPWM);
void SPWM(SVPWMStruct *S_IN,float a,float b,float c);
void dq0(float theta, float a, float b, float c, float *d, float *q);
void abc( float theta, float d, float q, float *a, float *b, float *c);

#endif
