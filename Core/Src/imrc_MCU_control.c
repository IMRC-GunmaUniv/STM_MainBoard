#include "imrc_MCU_control.h"
#include "imrc_ecan.h"
#include "stm32f4xx_hal.h"
#include <string.h>


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

static uint8_t move_body[3]= {0,0,0};
static uint8_t pre_move_body[3]= {0,0,0};

void MCU_move(int DIR,int spead,int reverse){
    if (spead > MCU_max_speed){
        spead = MCU_max_speed; 
    }

    move_body[0]=DIR;
    move_body[1]=spead;
    move_body[2]=reverse; 

    int isChanged = 0;
    if(move_body[0] != pre_move_body[0] || move_body[1] != pre_move_body[1] || move_body[2] != pre_move_body[2]){
        isChanged = 1;
    }
    if(isChanged){
        pre_move_body[0] = move_body[0]; //DIR
        pre_move_body[1] = move_body[1]; //SPREED 
        pre_move_body[2] = move_body[2]; //反転  

        //printf("MCU move: DIR: %d, Speed: %d\n\r", pre_move_body[0], pre_move_body[1]);

        ecan_sendPacketMtoU(MCU_move_hcan, 16, 1, 3, 0, 3, pre_move_body);
        isChanged = 0;
    }
}   

// void MCU_move_arm(CAN_HandleTypeDef *arm_hcan, int arm_unit_id, int DIR, int spead){
    

// }