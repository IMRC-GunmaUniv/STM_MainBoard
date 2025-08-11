#ifndef IMRC_RU_CONTROL
#define IMRC_RU_CONTROL

// IMRC LD-220MG servo motor
// Version 1.0

#include "stm32f4xx_hal.h"

void RU_control(CAN_HandleTypeDef *ptr_hcan,int RU_unit_id,int relay_No,int relay_State);



#endif