#ifndef IMRC_LD_MAIN_H
#define IMRC_LD_MAIN_H

#include "stm32f4xx_hal.h"


//PCデータ構造体
#define MODULE_NAME_LEN 16
#define TOPIC_NAME_LEN  32
#define MAX_DATA_NUM    20

typedef struct {
  char  module_name[MODULE_NAME_LEN];
  char  topic_name[TOPIC_NAME_LEN];
  int   data_len;
  float data[MAX_DATA_NUM];
} pc_uart;

void parse_csv_to_struct(char *csv, pc_uart *uart_data);

uint8_t clampTo255(float val);

void PC_printf(const char *fmt, ...);

int sent_to_PC(const char* data_type, const char* data_identify, const char* data);

#endif