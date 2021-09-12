#include "communication.h"
#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
/**
 * information frame:
 * Speed Should be 16bits data signed   [2][3]
 * Angle Should be 16bits data signed   [2][3]
 * Toque Should be 16bits data signed   [2][3]
 * PID Should be 24bits data signed.P 8bits;I 8bits;D 8bits;    [2][3][4]
 * PI Should be 16bits data signed.P 8bits;I 8bits;             [2][3]
 *
 * In Read data instruction.[2] will be used.
 *      0x0:speed;
 *      0x1:angle;
 *      0x2:torque;
 *      0x3:current state;
 * each unit is 8bits
 *
 *
 * *** Master Should Send in this frame ***
 * Head frame
 * [0]: ID                      0-7bits
 *      0xFC:then is to set the FOC_ID;
 *      0xFF:Debug Mode
 *
 * [1]: information mode.       8-15bis
 *      0x0:Read data;
 *      0x1:Set speed data;(Stand for working in speed mode)
 *      0x2:Set angle data;(Stand for working in angle mode)
 *      0x3:Set toque data;(Stand for working in torque mode)
 *      0x4:Set PID
 *      0x5:Set PI
 *      0x6:Start operation
 *      0x7:Stop operation
 *
 *      if [0] is equal to 0xFC,then this value will the value of FOC_ID
 *
 * Data bit:decoding according to head frame.
 * [2]:                         0-7bits
 * [3]:                         8-15bits
 * [4]:                         16-23bits
 * [5]:CRC
 *
 *
 * *** The device will send in this frame ***
 * [0]:ID
 * [1]: data type
 *      0x0:speed
 *      0x1:angle
 *      0x2:torque
 *      0x3:current state
 * [2]:                 0-7  bits
 * [3]:                 8-15 bits
 * [4]:ERROR_CODE
 *	1:Over_Current
 *	2:Over_Duty
 *	3:UnderVoltage
 * */


#define V_MAX		2000      //测试使用ve_des = 100
#define V_MIN		-2000
#define Iq_MAX	3         //测试使用iq_des = 0.05
#define Iq_MIN	-3
#define Angle_MAX		PI2
#define Angle_Min		0
#define Iabc_MAX		10
#define Iabc_MIN		-10

#define I_I_MAX			50    //测试使用  2.0
#define I_I_MIN     0

#define I_P_MAX     50    //测试使用  20
#define I_P_MIN			0

#define V_I_MAX     0     //测试使用  -5.0
#define V_I_MIN     -50

#define V_P_MAX			0     //在之前的测试中，这个PI数值为负数才可正常工作
#define V_P_MIN			-50   //测试使用  -10

#define A_Pmax      40    //测试使用  20
#define A_Imax      20    //测试使用  1
#define A_Dmax      20    //测试使用  10

uint8_t RecMas[6];
uint8_t TranMas[5];
extern FOC_Struct foc_handle;
extern CurrSenStruct Sense_handle;
CAN_TxHeaderTypeDef hCAN_TxHeader;
CAN_RxHeaderTypeDef hCAN_RxHeader;
CAN_FilterTypeDef hCAN_Filter;
uint32_t TxMailbox;
extern CAN_HandleTypeDef hcan;
/**
 * @brief: act as the instruction
 * */
uint8_t Massage_Process(uint8_t *ReMas,uint8_t *TrMas,FOC_Struct *foc)
{
  int ve_int = 0;
  int angle_int = 0;
  int torque_int = 0;
  int Para_Pint = 0;
  int Para_Iint = 0;
  int Para_Dint = 0;
  int SendFlag = 0;
  if((ReMas[0]!=FOC_ID))
  {
    if(ReMas[0]==0xFC)
    {
      FOC_ID = ReMas[1];
      Write_FOC_ID(FOC_ID);
    }else if(ReMas[0]==0xFF)  //Debug communication,Normal communication is forbidden
    {
      DMA_printf("ID=%d\r\n"
                 "ia=%f,ib=%f,ic=%f,iq=%f,iq_des=%f\r\n"
                 "ve=%f,ve_des=%f,angle=%f,angle_old=%f,angle_des=%f\r\n"
                 "V_P=%f,V_I=%f,I_P=%f,I_I=%f,A_P=%f,A_I=%f,A_D=%f\r\n"
                 "State=%d,working_state=%c,error_code=%d,V_Bus=%f\r\n",
                 FOC_ID
                 ,foc->ia,foc->ib,foc->ic,foc->iq,foc->iq_des,
                 foc->ve_act,foc->ve_des,foc->theta,foc->theta_old,foc->theta_des,
                 foc->Ve_P,foc->Ve_I,foc->I_P,foc->I_I,foc->A_P,foc->A_I,foc->A_D,
                 foc->state,foc->working_state,ERROR_CODE,foc->U_Bus);
    }
    return 0;//communication not correct
  }else
  {
    switch(ReMas[1])
    {
      case 0x0:
        Massage_Pack(ReMas[2],TrMas,foc);
        SendFlag = 1;
        break;
      case 0x1: //Set Speed data
        ve_int = (ReMas[2]<<8)|ReMas[3];
        foc->ve_des = uint_to_float(ve_int,V_MIN,V_MAX,16);
        foc->working_state = Speed;
        break;
      case 0x2: //Set Angle data
        angle_int = (ReMas[2]<<8)|ReMas[3];
        foc->theta_des = uint_to_float(angle_int,Angle_MAX,Angle_MAX,16);
        foc->working_state = Angle;
        break;
      case 0x3: //Set Torque data
        torque_int = (ReMas[2]<<8)|ReMas[3];
        foc->iq_des = uint_to_float(torque_int,Iq_MIN,Iq_MAX,16);
        foc->working_state = Torque;
        break;
      case 0x4: //Set PID
        Para_Pint = ReMas[2];
        Para_Iint = ReMas[3];
        Para_Dint = ReMas[4];
        if(foc->working_state == Angle)
        {
          foc->A_P = uint_to_float(Para_Pint,0,A_Pmax,8)*dt_50ms;
          foc->A_I = uint_to_float(Para_Iint,0,A_Imax,8)*dt;
          foc->A_D = uint_to_float(Para_Dint,0,A_Dmax,8)*dt_5ms;
        }else
        {
          foc->A_P = 0;
          foc->A_I = 0;
          foc->A_D = 0;
        }
        break;
      case 0x5: //Set PI
        Para_Pint = ReMas[2];
        Para_Iint = ReMas[3];
        switch(foc->working_state)
        {
          case Speed:
            foc->Ve_P = uint_to_float(Para_Pint,V_P_MIN,V_P_MAX,8)*dt_5ms;
            foc->Ve_I = uint_to_float(Para_Iint,V_I_MIN,V_I_MAX,8)*dt_5ms;
            break;
          case Torque:
            foc->I_P = uint_to_float(Para_Pint,I_P_MIN,I_P_MAX,8)*dt;
            foc->I_I = uint_to_float(Para_Iint,I_I_MIN,I_I_MAX,8)*dt;
            break;
          default:
            foc->I_P = 0;
            foc->I_I = 0;
            foc->Ve_P = 0;
            foc->Ve_I = 0;
            break;
        }
        break;
      case 0x6: //Start Operation
        foc->state = Moving;
        ERROR_CODE = 0;
        Start_Drv8302();
        break;
      case 0x7: //Stop Operation
        foc->state = Pending;
        Shut_down();
        break;
      default:
        foc->state = Pending;
        Shut_down();
        break;
    }
  }
  return SendFlag;
}

/**
 * @brief: Slave Pack the massage to send
 * */
void Massage_Pack(uint8_t MasType,uint8_t *Mas,FOC_Struct *foc)
{
  int ve_int = float_to_uint(foc->ve_act,V_MAX,V_MIN,16);
  int angle_int = float_to_uint(foc->theta,Angle_MAX,Angle_Min,16);
  int iq_int = float_to_uint(foc->iq,Iq_MAX,Iq_MIN,16);
//  uint8_t Mas[5]={};
  Mas[0] = FOC_ID;
  switch(MasType)
  {
    case 0x0:   //read speed value
      StoreTheMas(ve_int,0x0,&Mas[1]);
      break;
    case 0x1:
      StoreTheMas(angle_int,0x1,&Mas[1]);
      break;
    case 0x2:
      StoreTheMas(iq_int,0x2,&Mas[1]);
      break;
    case 0x3:
      Mas[1] = 0x3;
      Mas[2] = foc->state;
      Mas[3] = foc->working_state;
      break;
    default:
      break;
  }
  Mas[4] = ERROR_CODE;
}

void StoreTheMas(int val,uint8_t ValType,uint8_t *StorePlace)
{
  StorePlace[0] = ValType;
  StorePlace[1] = (val&0xFF00)>>8;
  StorePlace[2] = (val&0xFF);
}


void CAN_FilterInit(void)
{
  hCAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  hCAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
  hCAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
  hCAN_Filter.FilterIdHigh = 0x1FF<<5;
  hCAN_Filter.FilterIdLow = 0x00;
  hCAN_Filter.FilterMaskIdHigh = 0x1FF<<5;
  hCAN_Filter.FilterMaskIdLow = 0x00;
  hCAN_Filter.FilterActivation = ENABLE;
  hCAN_Filter.FilterBank =0;
  hCAN_Filter.SlaveStartFilterBank = 0;
  HAL_CAN_ConfigFilter(&hcan,&hCAN_Filter);
}

void CAN_RxHeader_Pack(void)
{
  hCAN_RxHeader.StdId = 0x00;
  hCAN_RxHeader.ExtId = 0x00;
  hCAN_RxHeader.IDE = CAN_ID_STD;
  hCAN_RxHeader.RTR = CAN_RTR_DATA;
  hCAN_RxHeader.DLC = 8;
  //hCAN_RxHeader.Timestamp = 0;
}

void CAN_TxHeader_Pack(void)
{
  hCAN_TxHeader.StdId = 0x1FF;
  hCAN_TxHeader.ExtId = 0;
  hCAN_TxHeader.IDE = CAN_ID_STD;
  hCAN_TxHeader.RTR = CAN_RTR_DATA;
  hCAN_TxHeader.DLC = 8;
  hCAN_TxHeader.TransmitGlobalTime = ENABLE;
}

void CAN_USER_Init(void)
{
  TxMailbox = CAN_TX_MAILBOX0;
  CAN_FilterInit();
  CAN_RxHeader_Pack();
  CAN_TxHeader_Pack();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    //注意串口发送的时候会有堵塞现象，因此在测试阶段只能通过人为的加延时来解决，而在实际通信时需要通过一应一答的方式来进行。
    if(Massage_Process(RecMas,TranMas,&foc_handle))
    {
      HAL_UART_Transmit_DMA(huart,TranMas,5);
    }
    HAL_UART_Receive_IT(huart,RecMas,6);
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
  HAL_CAN_GetRxMessage(hcan1,CAN_RX_FIFO0,&hCAN_RxHeader,RecMas);
  if(Massage_Process(RecMas,TranMas,&foc_handle))
  {
    HAL_CAN_AddTxMessage(hcan1,&hCAN_TxHeader,TranMas,&TxMailbox);
    //HAL_UART_Transmit_DMA(&huart1,TranMas,5);   //temp
  }
}


