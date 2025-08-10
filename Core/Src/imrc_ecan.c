#include "imrc_ecan.h"
#include "stm32f4xx_hal.h"
#include <string.h>

// IMRC ECAN(Ethernet CAN) Module
// Version 2.2

int my_unit_code = 0;
int my_unit_id = 0;

void ecan_pushFloatToPayload(uint8_t pl[], float fl, int idx){
    if(fl < 0){
        fl *= -1;
    }

    int num = (int)fl;
    int dec = (int) ((fl - (float)num) * 100 + 0.5f);

    pl[idx] = (uint8_t)num;
    pl[idx + 1] = (uint8_t)dec;
}

float ecan_parseFloatFromPayload(uint8_t pl[], int idx) {
    return (float)pl[idx] + ((float)pl[idx + 1]) / 100.0f;
}

uint32_t ecan_codeIdConvertToAddr(int unit_code, int unit_id, int isSendfromMain)
{
    uint32_t addr = 0;
    addr += unit_code << 5;
    addr += unit_id << 1;
    addr += isSendfromMain;

    return addr;
}

void ecan_addrConvertToCodeId(uint32_t addr, uint32_t *unit_code_ptr, uint32_t *unit_id_ptr, uint32_t *isSendFromMain_ptr)
{
    *unit_code_ptr = (addr & 0x7E0) >> 5;
    *unit_id_ptr = (addr & 0x1E) >> 1;
    *isSendFromMain_ptr = addr & 0x001;
}

void ecan_headerConvertToIdxEntry(uint8_t payload_header, uint32_t *payload_index_ptr, uint32_t *payload_entry_ptr)
{
    *payload_index_ptr = (payload_header >> 5) & 0x07;
    *payload_entry_ptr = payload_header & 0x1F;
}

uint8_t ecan_idxEntryConvertToHeader(uint32_t payload_index, uint32_t payload_entry)
{
    uint8_t left = (payload_index & 0x07) << 5;
    uint8_t right = payload_entry & 0x1F;

    return (left | right);
}

int ecan_init(int unit_code, int unit_id)
{
    if (0 <= unit_code && unit_code <= 63)
    {
        my_unit_code = unit_code;
    }
    else
    {
        return -1;
    }

    if (0 <= unit_id && unit_id <= 7)
    {
        my_unit_id = unit_id;
    }
    else
    {
        return -1;
    }

    return 0;
}

void ecan_setFilter(CAN_HandleTypeDef *ptr_hcan)
{
    CAN_FilterTypeDef filter;

    uint32_t fId1 = ecan_codeIdConvertToAddr(0, 0, 1) << 5;               // フィルターID1 Mainからの全ユニットブロキャス
    uint32_t fId2 = ecan_codeIdConvertToAddr(my_unit_code, 0, 1) << 5;       // フィルターID2 Mainからのユニットブロキャス
    uint32_t fId3 = ecan_codeIdConvertToAddr(my_unit_code, my_unit_id, 1) << 5; // フィルターID3 Mainからのプロセス
    uint32_t fId4 = 0x7FF << 5;                                   // フィルターID4 使わないから0x1...1で埋める

    filter.FilterIdHigh = fId1;                     // フィルターID1
    filter.FilterIdLow = fId2;                      // フィルターID2
    filter.FilterMaskIdHigh = fId3;                 // フィルターID3
    filter.FilterMaskIdLow = fId4;                  // フィルターID4
    filter.FilterScale = CAN_FILTERSCALE_16BIT;     // 16モード
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // FIFO0へ格納
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDLIST; // IDリストモード
    filter.SlaveStartFilterBank = 14;
    filter.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(ptr_hcan, &filter);
}

void ecan_setAllPassFilter(CAN_HandleTypeDef *ptr_hcan)
{
    CAN_FilterTypeDef filter;

    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0x0000;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.SlaveStartFilterBank = 14;
    filter.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(ptr_hcan, &filter);
}

void ecan_start(CAN_HandleTypeDef *ptr_hcan)
{
    HAL_CAN_Start(ptr_hcan);
    HAL_CAN_ActivateNotification(ptr_hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 受信割り込み有効化
}

int sendPacket(CAN_HandleTypeDef *ptr_hcan, int dest_unit_code, int dest_unit_id, int isSendFromMain, uint32_t ph_index, uint32_t ph_entry, int len_pl_body, uint8_t pl_body[])
{
    // 送信用インスタンス等
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint32_t len_pl = len_pl_body + 1;
    uint8_t TxData[len_pl];

    // 送信メールボックスに空きがあったら送信開始
    if (0 < HAL_CAN_GetTxMailboxesFreeLevel(ptr_hcan))
    {
        // 送信用インスタンスの設定
        TxHeader.StdId = ecan_codeIdConvertToAddr(dest_unit_code, dest_unit_id, isSendFromMain); // 受取手のCANのID
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.DLC = len_pl;
        TxHeader.TransmitGlobalTime = DISABLE;

        TxData[0] = ecan_idxEntryConvertToHeader(ph_index, ph_entry);

        for (int i = 0; i < len_pl; i++)
        {
            TxData[i + 1] = pl_body[i];
        }

        // CANメッセージを送信
        if (HAL_CAN_AddTxMessage(ptr_hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
        {
            // Error_Handler();
            return -1;
        }

        return 0;
    }
    else
    {
        return -2;
    }
}

int ecan_sendPacket(CAN_HandleTypeDef *ptr_hcan, uint32_t ph_index, uint32_t ph_entry, int len_pl_body, uint8_t pl_body[])
{
    // Unit -> Mainのパケットを送信
    return sendPacket(ptr_hcan, my_unit_code, my_unit_id, 0, ph_index, ph_entry, len_pl_body, pl_body);
}

int ecan_sendPacketUtoU(CAN_HandleTypeDef *ptr_hcan, int dest_unit_code, int dest_unit_id, uint32_t ph_index, uint32_t ph_entry, int len_pl_body, uint8_t pl_body[])
{
    // Unit -> Unitのパケットを送信
    // 使うな。Sniffer用。
    return sendPacket(ptr_hcan, dest_unit_code, dest_unit_id, 0, ph_index, ph_entry, len_pl_body, pl_body);
}

int ecan_sendPacketMtoU(CAN_HandleTypeDef *ptr_hcan, int dest_unit_code, int dest_unit_id, uint32_t ph_index, uint32_t ph_entry, int len_pl_body, uint8_t pl_body[])
{
    // Main -> Unitのパケットを送信
    // 使うな。Main用。
    return sendPacket(ptr_hcan, dest_unit_code, dest_unit_id, 1, ph_index, ph_entry, len_pl_body, pl_body);
}

int ecan_sendEmptyPacket(CAN_HandleTypeDef *ptr_hcan, uint32_t ph_index, uint32_t ph_entry)
{
    // Unit -> Mainの空パケットを送信

    uint8_t pl_body[] = {0};
    return sendPacket(ptr_hcan, my_unit_code, my_unit_id, 0, ph_index, ph_entry, 0, pl_body);
}

int ecan_sendEmptyPacketUtoU(CAN_HandleTypeDef *ptr_hcan, int dest_unit_code, int dest_unit_id, uint32_t ph_index, uint32_t ph_entry)
{
    // Unit -> Unitの空パケットを送信
    // 使うな。Sniffer用。

    uint8_t pl_body[] = {0};
    return sendPacket(ptr_hcan, dest_unit_code, dest_unit_id, 0, ph_index, ph_entry, 0, pl_body);
}

int ecan_sendEmptyPacketMtoU(CAN_HandleTypeDef *ptr_hcan, int dest_unit_code, int dest_unit_id, uint32_t ph_index, uint32_t ph_entry)
{
    // Main -> Unitの空パケットを送信
    // 使うな。Main用。

    uint8_t pl_body[] = {0};
    return sendPacket(ptr_hcan, dest_unit_code, dest_unit_id, 1, ph_index, ph_entry, 0, pl_body);
}
