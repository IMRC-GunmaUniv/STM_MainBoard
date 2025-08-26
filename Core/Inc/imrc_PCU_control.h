#ifndef IMRC_PCU_CONTROL_H
#define IMRC_PCU_CONTROL_H

// IMRC LD-220MG servo motor
// Version 1.0

#include "stm32f4xx_hal.h"

int PCU_Init(CAN_HandleTypeDef *ptr_hcan, int unit_id);
void PCU_relay_control(int relay_State); //引数:( 0 or 1 )
void PCU_voltage_cutoff(void); //電圧カットオフ     
void PCU_voltage_recovery(void); //電圧復帰
void PCU_survival_signal(uint32_t timeout);//生存信号送信

#endif