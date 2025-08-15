#ifndef CAN_CONTROL_CONV_H
#define CAN_CONTROL_CONV_H

#include <stdint.h>
#include <stdbool.h>

// ボタン番号定義
#define BTN_UP       0
#define BTN_DOWN     1
#define BTN_LEFT     2
#define BTN_RIGHT    3
#define BTN_A        4
#define BTN_B        5
#define BTN_X        6
#define BTN_Y        7
#define BTN_L1       8
#define BTN_R1       9
#define BTN_L2       10
#define BTN_R2       11
#define BTN_LS       12
#define BTN_RS       13

// 左スティック
#define STK_L_RIGHT      0
#define STK_L_UPRIGHT    1
#define STK_L_UP         2
#define STK_L_UPLEFT     3
#define STK_L_LEFT       4
#define STK_L_DOWNLEFT   5
#define STK_L_DOWN       6
#define STK_L_DOWNRIGHT  7

// 右スティック
#define STK_R_RIGHT      8
#define STK_R_UPRIGHT    9
#define STK_R_UP         10
#define STK_R_UPLEFT     11
#define STK_R_LEFT       12
#define STK_R_DOWNLEFT   13
#define STK_R_DOWN       14
#define STK_R_DOWNRIGHT  15

// スティック座標
#define STK_L_X    0 //X座標（横）
#define STK_L_Y    1 //Y座標（縦）
#define STK_R_X    2 //X座標（横）
#define STK_R_Y    3 //Y座標（縦）

// 関数プロトタイプ
void canCtrlConv_Init(uint8_t, uint8_t); // 初期化関数
void passCANCtrlData(uint8_t *_data); // CANデータを受け取る関数
void allBtnAxiState(void); //全てのボタン、スティックの状態を更新
bool getBtnState(uint8_t); //ボタンの状態を取得
uint8_t getAxiState(uint8_t); //スティックの状態を取得
int16_t getAxiCoord(uint8_t); //スティックの座標を取得
bool getIsCANReceived(void); // CAN受信フラグを取得
void printControllerState(void); //デバック用

//mainでは使用しない
uint8_t onlyLib_diagAxiScalar(uint8_t, uint8_t); //斜め入力のスカラー計算

#endif