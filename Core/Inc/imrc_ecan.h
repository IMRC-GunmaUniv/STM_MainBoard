#ifndef IMRC_ECAN_H
#define IMRC_ECAN_H

// IMRC ECAN(Ethernet CAN) Library
// Version 2.5

#include "stm32f4xx_hal.h"


int ecan_init(int my_unit_code, int my_unit_id);

void ecan_setFilter(CAN_HandleTypeDef *ptr_hcan);

void ecan_setFilter_advance(CAN_HandleTypeDef *ptr_hcan, uint32_t filterFIFOAssignment, uint32_t filterBank, uint32_t slaveStartFilterBank);

void ecan_setAllPassFilter(CAN_HandleTypeDef *ptr_hcan);

void ecan_setAllPassFilter_advance(CAN_HandleTypeDef *ptr_hcan, uint32_t filterFIFOAssignment, uint32_t filterBank, uint32_t slaveStartFilterBank);

void ecan_start(CAN_HandleTypeDef *ptr_hcan);

void ecan_start_advance(CAN_HandleTypeDef *ptr_hcan, uint32_t activeITs);

void ecan_stop(CAN_HandleTypeDef *ptr_hcan);

void ecan_stop_advance(CAN_HandleTypeDef *ptr_hcan, uint32_t activeITs);


int ecan_sendPacket(CAN_HandleTypeDef *ptr_hcan, uint32_t ph_index, uint32_t ph_entry, int len_pl_body, uint8_t pl_body[]);

int ecan_sendPacketUtoU(CAN_HandleTypeDef *ptr_hcan, int dest_unit_code, int dest_unit_id, uint32_t ph_index, uint32_t ph_entry, int len_pl_body, uint8_t pl_body[]);

int ecan_sendPacketMtoU(CAN_HandleTypeDef *ptr_hcan, int dest_unit_code, int dest_unit_id, uint32_t ph_index, uint32_t ph_entry, int len_pl_body, uint8_t pl_body[]);

int ecan_sendEmptyPacket(CAN_HandleTypeDef *ptr_hcan, uint32_t ph_index, uint32_t ph_entry);

int ecan_sendEmptyPacketUtoU(CAN_HandleTypeDef *ptr_hcan, int dest_unit_code, int dest_unit_id, uint32_t ph_index, uint32_t ph_entry);

int ecan_sendEmptyPacketMtoU(CAN_HandleTypeDef *ptr_hcan, int dest_unit_code, int dest_unit_id, uint32_t ph_index, uint32_t ph_entry);


void ecan_parsePacket(CAN_RxHeaderTypeDef *rxHeader, uint8_t rxData[], uint32_t *unit_code, uint32_t *unit_id, uint32_t *isSendFromMain, uint32_t *pl_index, uint32_t *pl_entry, uint32_t *pl_body_len, uint8_t *pl_body);

int ecan_idxEntryCompare(uint32_t real_idx, uint32_t real_entry, uint32_t idx, uint32_t entry);


void ecan_pushFloatToPayload(uint8_t pl[], float fl, int idx);

float ecan_parseFloatFromPayload(uint8_t pl[], int idx);

uint32_t ecan_codeIdConvertToAddr(int unit_code, int unit_id, int isSendfromMain);

void ecan_addrConvertToCodeId(uint32_t addr, uint32_t *unit_code_ptr, uint32_t *unit_id_ptr, uint32_t *isSendFromMain_ptr);

void ecan_headerConvertToIdxEntry(uint8_t payload_header, uint32_t *payload_index_ptr, uint32_t *payload_entry_ptr);

uint8_t ecan_idxEntryConvertToHeader(uint32_t payload_index, uint32_t payload_entry);


#endif