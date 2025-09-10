#ifndef IMRC_MCU_CONTROL_H
#define IMRC_MCU_CONTROL_H

// IMRC MCU Control Header File
// Version 1.0

#include "stm32f4xx_hal.h"
#include <stdbool.h>

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
void MCU_move(int DIR,int spead, int reverse,int slow_move); //移動

//---関東夏ロボコン2025---
void MCU_injection(CAN_HandleTypeDef *ptr_hcan, int unit_id ,int enable); //慣性命令


int MCU_arm_Init(CAN_HandleTypeDef *arm_hcan, int arm_unit_id, int *arm_position ,uint8_t *__arm_data, uint8_t *__arm_data_type);

#define null_position 0
#define injection_position 1
#define aim_position 2
#define catch_position 3
#define drag_position 4
//int MCU_arm_control(int command); //アーム制御命令
void see_MCU2_data(void);





#endif