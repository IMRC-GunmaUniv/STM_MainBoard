#ifndef IMRC_MCU_MOVE_H
#define IMRC_MCU_MOVE_H

// IMRC LD-220MG servo motor
// Version 1.0

#include "stm32f4xx_hal.h"

#define RIGHT 0
#define FRONT_RIGHT 1
#define FRONT 2
#define FRONT_LEFT 3 
#define LEFT 4
#define BUCK_LEFT 5
#define BUCK 6
#define BUCK_RIGHT 7
#define Stop 8

#define LEFT_ROTATE 9
#define RIGHT_ROTATE 10


int MCU_move_init(CAN_HandleTypeDef *ptr_hcan, int unit_id);
void MCU_move(int DIR, int speed);






#endif