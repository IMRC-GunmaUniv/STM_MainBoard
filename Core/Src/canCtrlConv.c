#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "canCtrlConv.h"

static uint8_t _btnState[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // ボタンの状態を格納する配列
static uint8_t _axiState[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // スティックの状態を格納する配列
static int16_t _axiCoord[4] = {0, 0, 0, 0}; // スティックの座標を格納する配列
static uint8_t _receiveData[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // データ受信用配列
static const int16_t _midpoint = 128; //中立位置の値
static uint8_t _stickResolution;
static uint8_t _deadzone;
static bool _isCANReceived = false; // CAN受信フラグ

/**
 * @brief CANコントローラの初期化
 * @param _setResolution スティックの分解能（スティックの中心からの分解能）
 * @param _setDeadzone 斜め入力デッドゾーンの範囲
 */
void canCtrlConv_Init(uint8_t _setResolution, uint8_t _setDeadzone)
{
  _stickResolution = _setResolution;
  _deadzone = _setDeadzone;
}


/**
 * @brief CANデータを受信する
 * @param _data 受信データ
 * @details 具体例
 * void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
 * {
 *   if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader_ESP, RxData_ESP) == HAL_OK)
 *   {
 *     passCANCtrlData(RxData_ESP);
 *   }
 * }
 */
void passCANCtrlData(uint8_t *_data)
{
  if(_data[0] == 0xC0)
  {
    _isCANReceived = true;
    for (int i = 0; i < 8; i++)
    {
      _receiveData[i] = _data[i];
    }
  }
}


/**
 * @brief ボタンとスティックの状態を更新する。常に読み込む必要あり。
 */
void allBtnAxiState()
{
  uint8_t _array_n[4] = {4, 2, 12, 10};
  int8_t _array_diff = -4;

  for (int i = 0; i <= 7; i++)
  {
    _btnState[i] = ((_receiveData[1] >> i) & 1); // 初期化
  }
  for (int i = 0; i <= 5; i++)
  {
    _btnState[8+i] = ((_receiveData[2] >> i) & 1); // 初期化
  }

  for(int i = 3; i <= 6; i++)
  {
    _axiCoord[i-3] = _receiveData[i] - _midpoint;
    if((i-3)%2 == 1) _axiCoord[i-3] *= -1;
    if(_isCANReceived == false) _axiCoord[i-3] = 0;
  }

  for(int i = 3; i <= 6; i++)
  {
    if((_receiveData[i] - _midpoint) < 0 && (_isCANReceived == true))
    {
      _axiState[_array_n[i-3]] = -1*(_receiveData[i] - _midpoint);
      _axiState[_array_n[i-3]+_array_diff] = 0;
    }
    else if((_receiveData[i] - _midpoint) > 0)
    {
      _axiState[_array_n[i-3]+_array_diff] = _receiveData[i] - _midpoint;
      _axiState[_array_n[i-3]] = 0;
    }
    else
    {
      _axiState[_array_n[i-3]] = 0; // 中立位置
      _axiState[_array_n[i-3]+_array_diff] = 0; // 中立位置
    }
    _array_diff *= (-1);
  }

  for(int i = 1; i <= 5; i+=2)
  {
    _axiState[i] = onlyLib_diagAxiScalar(i-1, i+1);
  }
  _axiState[7] = onlyLib_diagAxiScalar(6, 0);

  for(int i = 9; i <= 13; i+=2)
  {
    _axiState[i] = onlyLib_diagAxiScalar(i-1, i+1);
  }
  _axiState[15] = onlyLib_diagAxiScalar(14, 8);
}


/**
 * @brief 斜め入力、deadzoneの判定処理を行う
 * @param _arrayX スティックのX軸配列インデックス
 * @param _arrayY スティックのY軸配列インデックス
 */
uint8_t onlyLib_diagAxiScalar(uint8_t _arrayX, uint8_t _arrayY)
{
  uint8_t _scalar = 0;
  uint16_t _X = _axiState[_arrayX];
  uint16_t _Y = _axiState[_arrayY];

  // deadzone内処理 斜め入力は必ず0になる
  if(_X <= _deadzone && _Y <= _deadzone)
  {
    if(_X == _Y)
    {
      _axiState[_arrayX] = 0;
      _axiState[_arrayY] = 0;
      _X = 0;
      _Y = 0;
    }
    else if(_X > _Y)
    {
      _axiState[_arrayY] = 0;
      _Y = 0;
    }
    else if(_Y > _X)
    {
      _axiState[_arrayX] = 0;
      _X = 0;
    }
    return 0;
  }

  //deadzone外の処理
  if ((_X < 2*_Y) && (_Y < 2*_X) && (_X!=0 && _Y!=0)) //斜め入力の範囲を変えたい場合は、この式を変更
  {
    _scalar = 0.5 + sqrt(_X * _X + _Y * _Y);
    _axiState[_arrayX] = 0;
    _axiState[_arrayY] = 0;
  }
  else if(_X < _Y)
  {
    _axiState[_arrayX] = 0;
  }
  else if(_Y < _X)
  {
    _axiState[_arrayY] = 0;
  }

  return _scalar;
}


/**
 * @brief ボタンの状態を取得
 * @param _btn 状態を取得したいボタンの番号
 */
bool getBtnState(uint8_t  _btn)
{
  if(_btn >= 14)
  {
    return false; // 無効なボタン番号
  }
  else
  {
    return _btnState[_btn]; // ボタンの状態を返す
  }
}


/**
 * @brief スティックの状態を取得
 * @param _axi 状態を取得したいスティックの番号
 */
uint8_t getAxiState(uint8_t _axi)
{
  if (_axi >= 16)
  {
    return 0; // 無効なスティック番号
  }
  else
  {
    return _axiState[_axi]; // スティックの状態を返す
  }
}


/**
 * @brief スティックの座標を取得
 * @param _axis 取得したいスティックの座標番号（0: LX, 1: LY, 2: RX, 3: RY）
 */
int16_t getAxiCoord(uint8_t _axis)
{
  if (_axis >= 4)
  {
    return 0; // 無効なスティック座標番号
  }
  else
  {
    return _axiCoord[_axis]; // スティックの座標を返す
  }
}


/**
 * @brief CAN受信フラグを取得
 */
bool getIsCANReceived(void)
{
  return _isCANReceived; // CAN受信フラグを返す
}


/**
 * @brief デバッグ用のコントローラのすべての状態を表示
 */
void printControllerState(void)
{
  uint8_t _sum = 0;

  printf("canCtrlConv -> BTN: ");
  if (getBtnState(0)) printf("UP, ");
  if (getBtnState(1)) printf("DOWN, ");
  if (getBtnState(2)) printf("LEFT, ");
  if (getBtnState(3)) printf("RIGHT, ");
  if (getBtnState(4)) printf("A, ");
  if (getBtnState(5)) printf("B, ");
  if (getBtnState(6)) printf("X, ");
  if (getBtnState(7)) printf("Y, ");
  if (getBtnState(8)) printf("L1, ");
  if (getBtnState(9)) printf("R1, ");
  if (getBtnState(10)) printf("L2, ");
  if (getBtnState(11)) printf("R2, ");
  if (getBtnState(12)) printf("LS, ");
  if (getBtnState(13)) printf("RS, ");

  printf("STK: ");
  printf("LX=%4d, ", getAxiCoord(0));
  printf("LY=%4d, ", getAxiCoord(1));
  printf("RX=%4d, ", getAxiCoord(2));
  printf("RY=%4d, ", getAxiCoord(3));

  for (uint8_t i = 0; i <= 7; i++)
  {
    _sum += _axiState[i];
  }
  if (_sum > 0)
  {
    printf("L_");
    if (getAxiState(0) >= 1) printf("RIGHT=%d", getAxiState(0));
    if (getAxiState(1) >= 1) printf("UPRIGHT=%d", getAxiState(1));
    if (getAxiState(2) >= 1) printf("UP=%d", getAxiState(2));
    if (getAxiState(3) >= 1) printf("UPLEFT=%d", getAxiState(3));
    if (getAxiState(4) >= 1) printf("LEFT=%d", getAxiState(4));
    if (getAxiState(5) >= 1) printf("DOWNLEFT=%d", getAxiState(5));
    if (getAxiState(6) >= 1) printf("DOWN=%d", getAxiState(6));
    if (getAxiState(7) >= 1) printf("DOWNRIGHT=%d", getAxiState(7));
  }

  if(_sum > 0) printf(", ");
  _sum = 0;

  for (uint8_t i = 8; i <= 15; i++)
  {
    _sum += _axiState[i];
  }
  if (_sum > 0)
  {
    printf("R_");
    if (getAxiState(8) >= 1) printf("RIGHT=%d", getAxiState(8));
    if (getAxiState(9) >= 1) printf("UPRIGHT=%d", getAxiState(9));
    if (getAxiState(10) >= 1) printf("UP=%d", getAxiState(10));
    if (getAxiState(11) >= 1) printf("UPLEFT=%d", getAxiState(11));
    if (getAxiState(12) >= 1) printf("LEFT=%d", getAxiState(12));
    if (getAxiState(13) >= 1) printf("DOWNLEFT=%d", getAxiState(13));
    if (getAxiState(14) >= 1) printf("DOWN=%d", getAxiState(14));
    if (getAxiState(15) >= 1) printf("DOWNRIGHT=%d", getAxiState(15));
  }

  printf("\n\r");
}
