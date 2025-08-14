#ifndef IMRC_MCU_MOVE_H
#define IMRC_MCU_MOVE_H

// IMRC LD-220MG servo motor
// Version 1.0

#include "stm32f4xx_hal.h"

#define Stop 0
#define F 1
#define B 2
#define L 3
#define R 4
#define F_L 5
#define F_R 6
#define B_L 7
#define B_R 8

#define L_rotate 10
#define R_rotate 11

int MCU_move_init(CAN_HandleTypeDef *ptr_hcan, int unit_id);
void MCU_move(int DIR, int speed);






#endif