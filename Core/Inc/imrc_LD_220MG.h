#ifndef IMRC_LD_220MG_H
#define IMRC_LD_220MG_H

// IMRC LD-220MG servo motor
// Version 1.0

#include "stm32f4xx_hal.h"

void LD_220MG_SetAngle(TIM_HandleTypeDef *htim, uint32_t CHANNEL, int angle) ;
//サーボモータ制御　引数:(タイマー,チャンネル,角度[°])




#endif