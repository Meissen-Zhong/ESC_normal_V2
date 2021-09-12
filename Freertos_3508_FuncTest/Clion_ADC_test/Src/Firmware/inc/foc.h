#ifndef __FOC_H
#define __FOC_H

#include "main.h"
#include "currentsense.h"
#include "positionsensor.h"
#include "MotoConfig.h"
#include "SVPWM.h"


#define Flash_Address ((uint32_t)0x0800FFF8)
//PID未来建议进行外部调整与控制
//#define I_P 5.0f        //电流KP值
//#define I_I 0.5f         //电流KI值
//#define Ve_P 0.5f        //速度KP值
//#define Ve_I 2        //速度KI值
#define I_Max 5
#define Ve_Max 1.5f
#define IQ_Max 10.0f
#define A_Max	 1.0f

#define V_Max 24
#define V_Min 22      																	//V_Bus应该定义在别处，应该为24V

#define RELOAD			40960																	//高精度定时器
#define HALF_RELOAD	(RELOAD/2)
#define PRESCALE	0																			//这个是定时器的预分频
#define MAXDutyPersent  0.95f

#define dt  			0.00016f//((PRESCALE+1)*(RELOAD+1)/72000000)
#define dt_50ms		0.05f
#define dt_5ms		0.005f
#define	dt_500ms	0.5f
/*
两个反馈点：速度反馈点，直流两相反馈点。
实际量的获得：转角，速度，两相电流。
*/

typedef struct 
{
    SVPWMStruct *svpwm;
    float ia,ib,ic,ia_past,ib_past;             																			//三相直流电流，直接反馈进来的(需要写ADC操控程序)
    float iq,id,iq_des,id_des,iq_int,id_int,iq_past,iq_past2,id_past,id_past2;  																			//两相直流电流，直接反馈计算得到的，还有目标值。
    float iap,ibe;              																			        //转子两相电流。
    float Uq_des,Ud_des;        																			        //目标两相直流电压值
    float Uap_des,Ube_des;      																			        //目标两相交流值
    float Ua,Ub,Uc;             																			        //目标三相电压
    float theta,theta_old,theta_des,theta_offset,dtheta_tmep,theta_int;       //这个应该从角度传感器的结构体中进行读入并进行数据转换,历史角度值，用于计算速度。
    float ve_des,ve_act,ve_int;               															  //目标速度,实际速度
		float I_P,I_I,Ve_P,Ve_I,A_I,A_P,A_D;
		float theta_diff_his;
		int OC;																														        //over current
		float U_Bus;
		uint8_t working_state;
		uint8_t state;//pointer to move or not move
}FOC_Struct;

#define LENGTH					3
/***
 * @brief ERROR code: 0x0(OK),0x1(OVERCurrent),0x2(Overload),0x3(Input underVoltage)
 */
#define ERROR_CODE			    MOTOR_STATE[0]
#define CUR_REG			    MOTOR_STATE[1]
#define FOC_ID			    MOTOR_STATE[2]  //default value is 0

extern uint8_t MOTOR_STATE[LENGTH];
extern FOC_Struct foc_handle;
extern CurrSenStruct Sense_handle;
extern UART_HandleTypeDef huart1;
//extern TIM_HandleTypeDef htim1;

//Task LeveL
enum work{Pending = 1,Moving =2,CurrentTest=3};
enum workingMode{None = '0',Speed = '1',Angle ='2',Torque = '3'};
void FOC_task(void);
void FOC_GetSensorTask(void);
void FOCStruct_Init(FOC_Struct *foc,SVPWMStruct *svpwm);
void init_PID(FOC_Struct *foc);
void Write_FOC_ID(uint8_t id);
//Safty LeveL
void currentLimit(float ia,float ib,float ic,float threshold,int state);
void OverDuty_Detect(float dutyA,float dutyB,float dutyC,int state);
void InputUnder_voltage(float Voltage);
void Current_Test_Fuc(void);
void Set_HrtimDuty(int CHA,int CHB,int CHC);
//Data Acquire LeveL
void Actual_Speed_Acquire(FOC_Struct *foc,float DT);
void Actual_Current_Acquire(FOC_Struct *foc,CurrSenStruct *Sense);
void Actual_Angle_Acquire(FOC_Struct *foc);
//Control LeveL
void FOC_Trans(FOC_Struct *foc);
void Angle_Control(FOC_Struct *foc);
void tuner_PI(float Para_P,float Para_I,float *intergal_val,float *Desti_data,float *Current_data,float *New_Desti_data,float limit);
void tuner_PID(float Para_P,float Para_I,float Para_D,float *intergal_val,float *diff_his,float dif,float *New_Desti_data,float limit,float DT);
void linearize_dtc(float *dtc);
void Shut_down(void);
//Hardware Level
void Init_Drv8302(void);
void Start_Drv8302(void);
void Stop_Drv8302(void);

#endif
