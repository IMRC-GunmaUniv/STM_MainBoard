#include "imrc_MCU_control.h"
#include "imrc_ecan.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

#include "main.h"
#include <stdbool.h>
#include "imrc_RU_control.h"

static int MCU_move_unit_id = 0;
static CAN_HandleTypeDef *MCU_move_hcan;
static int MCU_max_speed = 0; // 最大速度

int MCU_move_Init(CAN_HandleTypeDef *ptr_hcan, int unit_id ,int max_speed){
    if (0 <= unit_id && unit_id <= 7)
    {
        MCU_move_unit_id = unit_id;
        MCU_move_hcan = ptr_hcan;
        MCU_max_speed = max_speed; // 最大速度を設定
    }
    else
    {
        return -1;
    }

    return 0;
}

static uint8_t move_body[4]= {0,0,0,0};
static uint8_t pre_move_body[4]= {0,0,0,0};

void MCU_move(int DIR,int spead,int reverse, int slow_move){
    if (spead > MCU_max_speed){
        spead = MCU_max_speed; 
    }

    move_body[0]=DIR;
    move_body[1]=spead;
    move_body[2]=reverse; 
    move_body[3]=slow_move;

    int isChanged = 0;
    if(move_body[0] != pre_move_body[0] || move_body[1] != pre_move_body[1] || move_body[2] != pre_move_body[2]){
        isChanged = 1;
    }
    if(isChanged){
        pre_move_body[0] = move_body[0]; //DIR
        pre_move_body[1] = move_body[1]; //SPREED 
        pre_move_body[2] = move_body[2]; //反転  
        pre_move_body[3] = move_body[3]; // 

        //printf("MCU move: DIR: %d, Speed: %d\n\r", pre_move_body[0], pre_move_body[1]);

        ecan_sendPacketMtoU(MCU_move_hcan, 16, 1, 3, 0, 4, pre_move_body);
        isChanged = 0;
    }
}   

//---関東夏ロボコン2025---
int Last_is_injection_enable;
void MCU_injection(CAN_HandleTypeDef *ptr_hcan, int unit_id ,int is_injection_enable){//廃止
    if(is_injection_enable == Last_is_injection_enable){
        return; 
    }else{
        uint8_t body[4] = {8,0,0,is_injection_enable};
        printf("MCU injection command: %d\n\r", body[3]);
        ecan_sendPacketMtoU(ptr_hcan, 16, unit_id, 3, 0, 4, body);
        Last_is_injection_enable = is_injection_enable;
    }
}

static int MCU_arm_unit_id = 0;
static CAN_HandleTypeDef *MCU_arm_hcan;
static int *MCU_arm_position = NULL; // 最大速度
static uint8_t *arm_data = NULL;
static uint8_t *arm_data_type = NULL;
int MCU_arm_Init(CAN_HandleTypeDef *arm_hcan, int arm_unit_id, int *arm_position ,uint8_t *__arm_data, uint8_t *__arm_data_type){
    if (0 <= arm_unit_id && arm_unit_id <= 7){
        MCU_arm_hcan = arm_hcan;
        MCU_arm_unit_id = arm_unit_id;
        MCU_arm_position = arm_position;
        arm_data =  __arm_data;
        arm_data_type =  __arm_data_type;
        
    }else{
        return -1;
    }

    return 0;

}


// int MCU_arm_control(int command){
//     static int is_moving = 0;
//     static int last_arm_command = 0;

//     if(command == last_arm_command){
//         return false; // コマンドが前回と同じ場合は何もしない
//     }else if(command == MCU_arm_position){
//         printf("Already this position\n\r");
//         return 1;
//     }else{
//         uint8_t body[1] = {command};
//         printf("send MCU arm command: %d\n\r", body[0]);
//         ecan_sendPacketMtoU(MCU_arm_hcan, 16, MCU_arm_unit_id, 3, 0, 1, body);
//         last_arm_command = command;
//         is_moving = 1;
//     }

//     if(is_moving == 1){
//         printf("aaa");
//         if(arm_data_type[0] == 3 && arm_data_type[1] == 1 && arm_data[1] == command){
//             is_moving = 0;
//             *MCU_arm_position = command;
//             //printf("moving done \t current position:%d\n\r",*MCU_arm_position);
//             return 1;
//         }else{
//             printf("moving to position %d\n\r",command);
//         }
        
//     }
//     return 0;
// }

void see_MCU2_data(void){
    printf("index:%d \t entry:%d data: ",arm_data_type[0],arm_data_type[1]);
    for(int i=1 ; i<7 ;i++){
        printf("%d",arm_data[i]);
    }
    printf("\n\r");
}



// void MCU_move_arm(CAN_HandleTypeDef *arm_hcan, int arm_unit_id, int DIR, int spead){
    

// }