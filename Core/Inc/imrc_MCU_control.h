#ifndef IMRC_MCU_CONTROL_H
#define IMRC_MCU_CONTROL_H

// IMRC MCU Control Header File
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


int MCU_move_Init(CAN_HandleTypeDef *ptr_hcan, int unit_id, int max_speed);//移動MCU基板　定義
void MCU_move(int DIR,int spead, int reverse); //移動

//---関東夏ロボコン2025---
void MCU_injection(CAN_HandleTypeDef *ptr_hcan, int unit_id ,int enable); //慣性命令




#endif