#include "imrc_LD_220MG.h"
#include "stm32f4xx_hal.h"
#include <string.h>

// IMRC LD-220MG servo motor
// Version 1.0

void LD_220MG_SetAngle(TIM_HandleTypeDef *htim, uint32_t CHANNEL, int angle){//サーボモータ制御　引数:(タイマー,チャンネル,角度[°])

    uint16_t ccr_value = 500 + (angle * (2000)) / 180;

    if (ccr_value < 500) {
        ccr_value = 500; // 最小値の制限
    } else if (ccr_value >= 2450) {
        ccr_value = 2450; // 最大値の制限
    }

    __HAL_TIM_SET_COMPARE(htim, CHANNEL, ccr_value);

}