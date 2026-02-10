#ifndef IMRC_CONNECT_MONITOR_H
#define IMRC_CONNECT_MONITOR_H

#include "stm32f4xx_hal.h"

typedef struct {
    const char *name;          // ユニット名（ログ用）
    uint8_t unit_no;           // ID
    uint8_t connected;         // 0: NG, 1: OK
    uint32_t last_rx_time;     // 最終受信時刻
    uint32_t timeout_ms;       // タイムアウト時間
} unit_t;

void conn_init(unit_t *units, uint8_t count,int mode);//初期化関数 
//第一引数に接続するユニットを入れた配列を入れる
//↓例
// unit_t units[] = {
//     { "MOTOR",  1, 0, 0, 100 },
//     { "IMU",    2, 0, 0, 50  },
//     { "ENC",    3, 0, 0, 20  },
// };

//第二引数に接続するユニットの数を入れる
//第三引数にチェックモードを入れる。　
//　0:定義したユニットをチェック　1:最初につながっていたユニットだけを見る


void conn_rx(uint8_t unit_no, uint32_t now_ms); //CAN割り込みなどの受信部に入れる
//unit_noはuint_tの第二引数のもの

void conn_update(uint32_t now_ms);  //つながっているかを判定する　ループ内で使う

uint8_t conn_is_connected(uint8_t unit_no); //つながっているかを取得する

void conn_chack();  //もしつながっていなかったら...の処理

void conn_state(); //つながっていないユニットを表示

#endif