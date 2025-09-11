#include "imrc_PCU_control.h"
#include "stm32f4xx_hal.h"
#include "imrc_ecan.h"
#include <stdio.h>
#include <string.h>
#include "main.h"

// IMRC PCU control
// Version 1.0


static int PCU_unit_id = 0;
static CAN_HandleTypeDef *PCU_hcan;

int PCU_Init(CAN_HandleTypeDef *ptr_hcan, int unit_id){
    if (0 <= unit_id && unit_id <= 7)
    {
        PCU_unit_id = unit_id;
        PCU_hcan = ptr_hcan;
    }
    else
    {
        return -1;
    }

    return 0;
}

static int pre_State=0;

void PCU_relay_control(int relay_State){ //引数:( 0 or 1 )
    int isChanged = 0;
    if(relay_State != pre_State){
        isChanged = 1;

    }

    if(isChanged){
        pre_State = relay_State; // 前回の状態を更新
        uint8_t body[1] = { (uint8_t)relay_State };
        if(body[0]==0){
            printf("PCU_voltage_cutoff\n\r");
        }else if(body[0]==1){
            printf("PCU_voltage_recovery\n\r");
        }
        ecan_sendPacketMtoU(PCU_hcan, 18, PCU_unit_id, 3, 0, 1, body);

        
    } 
}


static int State=0;

void PCU_voltage_cutoff(void){ 
    State = 0;
    PCU_relay_control(State);
}

void PCU_voltage_recovery(void){ 
    State = 1;
    PCU_relay_control(State);
}

void PCU_survival_signal(uint32_t timeout){
    if(timeout == 0){
        ecan_sendEmptyPacketMtoU(PCU_hcan, 18, 1, 0, 5);

    }else if(timeout > 0){
        static uint32_t last_signal_time = 0;
        if (HAL_GetTick() - last_signal_time >= timeout) {
            ecan_sendEmptyPacketMtoU(PCU_hcan, 18, 1, 0, 5);
            //printf("PCU survival signal sent\n\r");
            last_signal_time = HAL_GetTick();
        }

    }


    
    
}