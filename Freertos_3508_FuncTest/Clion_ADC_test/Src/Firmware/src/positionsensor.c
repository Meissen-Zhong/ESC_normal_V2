#include "positionsensor.h"
#include "lut.h"
#include "math.h"
#ifdef MA700_positionSenser
extern SPI_HandleTypeDef hspi1;

/**
 * @brief 底层的spi操作函数
*/
void TransRece(uint8_t *write,uint8_t *read)
{
	HAL_GPIO_WritePin(MA700_CS_Port,MA700_CS_Pin,GPIO_PIN_RESET);
	
	HAL_SPI_TransmitReceive(&hspi1,write,read,3,0xff);
	
	HAL_GPIO_WritePin(MA700_CS_Port,MA700_CS_Pin,GPIO_PIN_SET);
}

/**
 * @brief 通过SPI获得角度值，传感器为MA700
*/
uint16_t Get_RawAngle(void)
{
	uint8_t read[3];
	uint8_t write[3];
	write[0] = 0;
	write[1] = 0;
	write[2] = 0;
	uint16_t rawAngle=0;
	HAL_GPIO_WritePin(MA700_CS_Port,MA700_CS_Pin,GPIO_PIN_RESET);
	
	HAL_SPI_TransmitReceive(&hspi1,write,read,3,0xFF);
	
	HAL_GPIO_WritePin(MA700_CS_Port,MA700_CS_Pin,GPIO_PIN_SET);
	
	rawAngle = (uint16_t)read[0]<<6;
	rawAngle |= (uint16_t)read[1] >>2;
	return rawAngle;
}

/**
 * @brief 对原始的角度值进行处理
*/
float Angle_Get(void)
{
	int i;
	float angle=0;
	for(i = 0;i<1;i++)
	{
		angle += (float)Get_RawAngle()/M*PI2/1;
	}
	return angle;
}
#endif
#ifdef TAN_positionSenser

extern ADC_HandleTypeDef hadc2;
//float fast_atan(float val)
//{
//  int Num = val*SIZE;
//  return atan_Table[Num];
//}

//double FastArcTan(double x) {
//  return PI_4*x - x*(fabs(x) - 1)*(0.2447 + 0.0663*fabs(x));
//}

double atan2LUT(double y,double x)
{
  double absx, absy;
  absy = fabs(y);
  absx = fabs(x);
  short octant = ((x<0) << 2) + ((y<0) << 1 ) + (absx <= absy);
  switch (octant) {
    case 0: {
      if (x == 0 && y == 0)return 0;
      return ATAN_LUT[(int)(255*absy/absx)]; //1st octant
//      break;
    }
    case 1:{
      if (x == 0 && y == 0)
        return 0.0;
      return PI_2 - ATAN_LUT[(int)(255*absx/absy)]; //2nd octant
//      break;
    }
    case 2: {
      return -ATAN_LUT[(int)(255*absy/absx)]; //8th octant
//      break;
    }
    case 3: {
      return -PI_2 + ATAN_LUT[(int)(255*absx/absy)];//7th octant
//      break;
    }
    case 4: {
      return  PI - ATAN_LUT[(int)(255*absy/absx)];  //4th octant
    }
    case 5: {
      return  PI_2 + ATAN_LUT[(int)(255*absx/absy)];//3rd octant
//      break;
    }
    case 6: {
      return -PI + ATAN_LUT[(int)(255*absy/absx)]; //5th octant
//      break;
    }
    case 7: {
      return -PI_2 - ATAN_LUT[(int)(255*absx/absy)]; //6th octant
//      break;
    }
    default:
      return 0.0;
  }
}

void TestAngleADC(uint32_t *Val)
{
  HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc2,Val,2);
  HAL_ADC_Start_DMA(&hadc2,Val,2);
}

float Angle_Get(void)
{
  uint32_t ConverVal[2];
  float TransVal[2];
  float angle;
#ifdef Avarege_Filter
  uint32_t temp_Val[2] = {};
  int i,num=100;
  for(i = 0;i<num;i++)
  {
    HAL_ADC_Start_DMA(&hadc2,ConverVal,2);
    HAL_ADC_Start_DMA(&hadc2,ConverVal,2);
    temp_Val[0] += ConverVal[0]/num;
    temp_Val[1] += ConverVal[1]/num;
  }
//  TransVal[0] = (((float)ConverVal[0]/4096*3.3f)-0.97485f);//view as A:cos channel 1
//  TransVal[1] = (((float)ConverVal[1]/4096*3.3f)-0.97485f);//view as B:sin	channel 3
  TransVal[0] = (((float)temp_Val[0]/4096*3.3f)-0.97485f);//view as A:cos channel 1
  TransVal[1] = (((float)temp_Val[1]/4096*3.3f)-0.97485f);//view as B:sin	channel 3
#endif
#ifdef LeveL1_Filter
  float fil_para = 0.6f;
  uint32_t temp_val[2];
  HAL_ADC_Start_DMA(&hadc2,ConverVal,2);
  HAL_ADC_Start_DMA(&hadc2,ConverVal,2);
  temp_val[0] = ConverVal[0];
  temp_val[1] = ConverVal[1];
  HAL_ADC_Start_DMA(&hadc2,ConverVal,2);
  HAL_ADC_Start_DMA(&hadc2,ConverVal,2);
  ConverVal[0] = fil_para*ConverVal[0]+(1-fil_para)*temp_val[0];
  ConverVal[1] = fil_para*ConverVal[1]+(1-fil_para)*temp_val[1];
  TransVal[0] = (((float)ConverVal[0]/4096*3.3f)-0.97485f);//view as A:cos channel 1
  TransVal[1] = (((float)ConverVal[1]/4096*3.3f)-0.97485f);//view as B:sin	channel 3
#endif
#ifdef Weight_Filter    //效果还行
  uint32_t temp_Val[2];
	temp_Val[0] = 0;
	temp_Val[1] = 0;
  int i,num=14;
//  int core[12] = {1,2,3,4,5,6,7,8,9,10,11,12};
  int core_sum = 105;
  for(i = 1;i<=num;i++)
  {
		//HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc2,ConverVal,2);
    HAL_ADC_Start_DMA(&hadc2,ConverVal,2);
    temp_Val[0] += ConverVal[0]*i/core_sum;
    temp_Val[1] += ConverVal[1]*i/core_sum;
  }
  TransVal[0] = (((float)temp_Val[0]/4096*3.3f)-0.97485f);//view as A:cos channel 1
  TransVal[1] = (((float)temp_Val[1]/4096*3.3f)-0.97485f);//view as B:sin	channel 3
#endif
  angle=atan2LUT(TransVal[1],TransVal[0])+PI;
  return angle;
}

//手动测量：
void GetOffSet(int *Max,int *Min)
{
  uint32_t TempVal[2];
  for(int i = 0;i<10000;i++)
  {
    TestAngleADC(TempVal);
    if(TempVal[0]>*Max)
    {
      *Max = TempVal[0];
    }
    if(TempVal[0]<*Min)
    {
      *Min = TempVal[0];
    }
  }
}

#endif

