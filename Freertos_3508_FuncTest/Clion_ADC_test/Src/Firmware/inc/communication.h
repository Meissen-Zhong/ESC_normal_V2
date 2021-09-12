#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "Cmath.h"
#include "foc.h"
uint8_t Massage_Process(uint8_t *ReMas,uint8_t *TrMas,FOC_Struct *foc);
void Massage_Pack(uint8_t MasType,uint8_t *Mas,FOC_Struct *foc);
void StoreTheMas(int val,uint8_t ValType,uint8_t *StorePlace);
void CAN_USER_Init(void);
#endif
