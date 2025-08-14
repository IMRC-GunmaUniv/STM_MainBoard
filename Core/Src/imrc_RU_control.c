#include "imrc_RU_control.h"
#include "stm32f4xx_hal.h"
#include "imrc_ecan.h"
#include <stdio.h>
#include <string.h>

// IMRC RU control
// Version 1.0


uint8_t relayState[4] = {0, 0, 0, 0}; // リレーの状態を格納する配列
uint8_t pre_relayState[4] = {0, 0, 0, 0}; // 前回のボタン状態を格納する配列

void RU_control(CAN_HandleTypeDef *ptr_hcan,int RU_unit_id,int relay_No,int relay_State){ //引数:(CANハンドル, ユニットID, リレー番号, 0 or 1 )
  if (relay_No < 1 || relay_No > 4) {
    printf("Invalid relay number: %d\n\r", relay_No);
    return; // 無効なリレー番号
  }else{
    relayState[relay_No - 1] = relay_State; // relayState配列に値を格納
  }       


  int isChanged = 0;
  for (int i = 0; i < 4; i++){
    if(relayState[i] != pre_relayState[i]){
      isChanged = 1;
      break;

    }
  }

  if(isChanged){
    for (int i = 0; i < 4; i++)
    {
      uint8_t body[] = {i+1, relayState[i]};
      ecan_sendPacketMtoU(ptr_hcan, 19, RU_unit_id, 3, 0, 2, body);

      pre_relayState[i] = relayState[i];
      
      HAL_Delay(1);
    } 
  }
}