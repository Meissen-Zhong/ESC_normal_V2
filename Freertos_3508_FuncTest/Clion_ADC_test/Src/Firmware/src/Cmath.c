#include "Cmath.h"
#include "lut.h"
#include "stdio.h"
#include "math.h"
float state[FIR_FILTER_LENGTH+1]={0,0,0,0,0};
const float FIR_COFFES[FIR_FILTER_LENGTH] = {
    -0.2426067468255,   0.2000121578697,   0.7793774314732,   0.2000121578697,
    -0.2426067468255
};
int M;
const int Val = Lut_Num/4;
float SinTable(float theta)
{
	while(theta > PI2)
	{
		theta -= PI2;
	}
	M = (int)(theta/PI2*Lut_Num);
	return sinlut1[M];
}


float CosTable(float theta)
{
	while(theta > PI2)
	{
		theta -= PI2;
	}
	return SinTable(theta + 1.57079632679f);
}

float fmaxf(float x, float y){
    /// Returns maximum of x, y ///
    return (((x)>(y))?(x):(y));
    }

float fminf(float x, float y){
    /// Returns minimum of x, y ///
    return (((x)<(y))?(x):(y));
    }

float fmaxf3(float x, float y, float z){
    /// Returns maximum of x, y, z ///
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
    }

float fminf3(float x, float y, float z){
    /// Returns minimum of x, y, z ///
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
    }


void limit_norm(float *x, float *y, float limit){
    float norm = sqrt(*x * *x + *y * *y);
    if(norm > limit){
        *x = *x * limit/norm;
        *y = *y * limit/norm;
        }
    }

//void LowPassFilter_RC_1order(float *Vi, float *Vo, float *Vo_p, float *Vo_p2,double sampleFrq ,float CutFrq)
//{
//  float  RC, Cof1;
//    
//  //low pass filter @cutoff frequency = 1 Hz    
//  RC = (float)1.0f/2.0f/PI/CutFrq;
//  Cof1 = 1/(1+RC*sampleFrq);
//  //Cof2 = RC*sampleFrq/(1+RC*sampleFrq);// no problem
//  *Vo = 0.67f * (*Vi) + 0.33f * (*Vo_p);    
//  
//  //update  
//	*Vo_p2 = *Vo_p;
//  *Vo_p = *Vo;    
//}
void LowPassFilter_RC_1order(float *Vi, float *Vo, float *Vo_p, double sampleFrq ,float CutFrq)
{
  float  RC, Cof1;
    
  //low pass filter @cutoff frequency = 1 Hz    
  RC = (float)1.0f/2.0f/PI/CutFrq;
  Cof1 = 1/(1+RC*sampleFrq);
  //Cof2 = RC*sampleFrq/(1+RC*sampleFrq);// no problem
  *Vo = Cof1 * (*Vi) + (1.0f-Cof1) * (*Vo_p);    
  
  //update  
  *Vo_p = *Vo;    
}

int float_to_uint(float x, float x_min, float x_max, int bits){
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits){
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

float uint8_to_float(uint8_t *BUFFER)
{
	uint8_t lenght = 2;
	uint16_t mask = 1<<(lenght*8-1);
	uint16_t Total = BUFFER[0]|(BUFFER[1]<<8);
	printf("%x",Total);
	float ret;
	if(mask&Total)
	{
		Total ^= mask;
		ret = -(float)Total;
	}else{
		ret = (float)Total;
	}
	return ret;
}

//CRC8校验
//ptr:要校验的数组
//len:数组长度
//返回值:CRC8码
//多项式0X31,LSB First，初始值0X00
uint8_t Get_Crc8(uint8_t *ptr,uint16_t len)
{
  uint8_t crc;
  uint8_t i;
  crc=0;
  while(len--)
  {
    crc^=*ptr++;
    for(i=0;i<8;i++)
    {
      if(crc&0x01)crc=(crc>>1)^0x8C;
      else crc >>= 1;
    }
  }
  return crc;
}

void Matlab_FIR(float *x,float *y,int Num)
{
	int i = 0,j = 0,k=0;
	float temp; 
	for (k = 0; k < Num; k++) 
	{ 
		state[0] = x[k];
		for (i = 0, temp = 0; i < FIR_FILTER_LENGTH; i++)
			temp += FIR_COFFES[i] * state[i];
		y[k] = temp;
		for (j = FIR_FILTER_LENGTH - 1; j > -1 ; j--)
		state[j+1] = state[j];
	}
}
