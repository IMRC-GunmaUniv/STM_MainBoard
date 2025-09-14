#include "imrc_RU_control.h"
#include "stm32f4xx_hal.h"
#include "imrc_ecan.h"
#include <stdio.h>
#include <string.h>
//#include <stdbool.h>


// IMRC RU control
// Version 1.0


uint8_t relayState[4] = {0, 0, 0, 0}; // リレーの状態を格納する配列
uint8_t pre_relayState[4] = {0, 0, 0, 0}; // 前回のボタン状態を格納する配列

bool RU_control(CAN_HandleTypeDef *ptr_hcan,int RU_unit_id,int relay_No,int relay_State){ //引数:(CANハンドル, ユニットID, リレー番号, 0 or 1 )
  if (relay_No < 1 || relay_No > 4) {
    printf("Invalid relay number: %d\n\r", relay_No);
    return; // 無効なリレー番号
  }else{
    relayState[relay_No - 1] = relay_State; // relayState配列に値を格納
  }       

  for (int i = 0; i < 4; i++){
    if(relayState[i] != pre_relayState[i]){
      uint8_t body[] = {i+1, relayState[i]};
      ecan_sendPacketMtoU(ptr_hcan, 19, RU_unit_id, 3, 0, 2, body);

      pre_relayState[i] = relayState[i];
      printf("RU Relay %d: %d\n\r", body[0], body[1]);
      
      HAL_Delay(1);
    }
    
  } 

  return 1;
}

void RU_Toggle_relay(CAN_HandleTypeDef *ptr_hcan, int RU_unit_id, int relay_No, uint32_t ontime, uint32_t offtime)
{
  static uint32_t last_tick = 0;
  uint32_t now = HAL_GetTick();
  static int state = 0; // 0 or 1

  uint32_t timeout = (state == 0) ? ontime : offtime;

  if ((now - last_tick) >= timeout) {
    state = !(state);    // 0 ⇔ 1 を切り替え
    last_tick = now;       // 最終更新時刻を更新

    uint8_t body[] = {relay_No, state};
    ecan_sendPacketMtoU(ptr_hcan, 19, RU_unit_id, 3, 0, 2, body);

    printf("RU Toggle relay %d: %d\n\r", body[0], body[1]);
  }

  
  
}