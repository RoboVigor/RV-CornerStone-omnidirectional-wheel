#ifndef _DRIVER_VOFA_H
#define _DRIVER_VOFA_H

#include "stm32f4xx_conf.h"
#include "stm32f4xx_can.h"
#include "Driver_Motor.h"
#include "Driver_Protocol.h"
#include "Driver_CAN.h"
#include "Driver_Bridge.h"
#include "Driver_BSP.h"
#include "config.h"
#include "handle.h"

#define vofaTail_t 0x807F

void Vofa_Send(uint16_t frequency, uint16_t heapDeth, Node_Type * node, uint32_t commandID);
void Task_Vofa_Send(void * Parameters);
#endif 
