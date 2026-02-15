#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"
#include "usart.h"

#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include <stdarg.h>
#include <stdint.h>
#include <string.h>

#include "imrc_ecan.h"
#include "imrc_main.h"


void parse_csv_to_struct(char *csv, pc_uart *uart_data){
  char *token;
  int field = 0;
  int data_index = 0;

  token = strtok(csv, ",");

  while (token != NULL) {

    switch (field) {
      case 0: // module_name
        strncpy(uart_data->module_name, token, MODULE_NAME_LEN - 1);
        uart_data->module_name[MODULE_NAME_LEN - 1] = '\0';
        break;

      case 1: // topic_name
        strncpy(uart_data->topic_name, token, TOPIC_NAME_LEN - 1);
        uart_data->topic_name[TOPIC_NAME_LEN - 1] = '\0';
        break;

      case 2: // data_len
        uart_data->data_len = atoi(token);
        if (uart_data->data_len > MAX_DATA_NUM) {
          uart_data->data_len = MAX_DATA_NUM;
        }
        break;

      default: // data[]
        if (data_index < uart_data->data_len) {
          uart_data->data[data_index++] = atof(token);
        }
        break;
    }

    field++;
    token = strtok(NULL, ",");
  }
}

uint8_t clampTo255(float val){
  int clampVal = 128 + val * 128;

  if(clampVal > 255 ) clampVal = 255;
  if(clampVal < 0 ) clampVal = 0;
  

  return (uint8_t)clampVal;
}

void PC_printf(const char *fmt, ...)
{
  char buf[128];   // 必要に応じてサイズ調整
  va_list args;

  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), 100);
}

int sent_to_PC(const char* data_type, const char* data_identify, const char* data){
  char buffer[256];
  int offset = 0;

  // 先頭2つ（module_name, topic_name）
  offset += snprintf(buffer + offset, sizeof(buffer) - offset,"%s,%s,%s", data_type, data_identify, data);

  // 最後に改行
  snprintf(buffer + offset, sizeof(buffer) - offset, "\n\r");

  PC_printf("%s", buffer);

  return 0;
}
