#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "imrc_connect_monitor.h"
#include "stm32f4xx_hal.h"
#include "imrc_main.h"

static unit_t *unit_list;
static uint8_t unit_count;
static int chack_mode;

void conn_init(unit_t *units, uint8_t count,int mode) {//つなげるユニットを定義、初期化
    unit_list  = units;
    unit_count = count;
	chack_mode = mode;

    for (int i = 0; i < count; i++) {//初期化
        unit_list[i].connected = 0;
        unit_list[i].last_rx_time = 0;
    }
}

void conn_rx(uint8_t unit_no, uint32_t now_ms) {
    for (int i = 0; i < unit_count; i++) {

        if (unit_list[i].unit_no == unit_no) {
            unit_list[i].last_rx_time = now_ms;
            unit_list[i].connected = 1;
            return;
        }
    }
}

void conn_update(uint32_t now_ms) { //use in main loop
    for (int i = 0; i < unit_count; i++) {
        if (unit_list[i].connected && (now_ms - unit_list[i].last_rx_time > unit_list[i].timeout_ms)) {
            unit_list[i].connected = 0;
        }
    }
}

uint8_t conn_is_connected(uint8_t unit_no) {
    for (int i = 0; i < unit_count; i++) {
        if (unit_list[i].unit_no == unit_no) {
            return unit_list[i].connected;
        }
    }
    return 0;
}

void conn_chack(){
	for (int i = 0; i < unit_count; i++) {
		if(chack_mode == 0 &&  unit_list[i].connected == 0){
			printf("unit disconnected: %s \n\r",unit_list[i].name);

            offset += snprintf(buffer + offset, sizeof(buffer) - offset,"%s,%s,%s", data_type, data_identify, data);
            sent_to_PC("ERR", "unit_indo", data);
			
		}else if(chack_mode == 1 && unit_list[i].connected == 0 && unit_list[i].last_rx_time > 0){
			printf("unit disconnected: %s \n\r",unit_list[i].name);

		}
	}
}

void conn_state(){
	printf("-------------------------------\n\r");
	for (int i = 0; i < unit_count; i++) {
		if(unit_list[i].connected == 0){
			printf("%s :not connected\n\r",unit_list[i].name);

		}
	}
	printf("-------------------------------\n\r");
}
