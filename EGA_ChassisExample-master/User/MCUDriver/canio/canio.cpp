#include <cstring>
#include "canio.hpp"
#include "chassisc.hpp"
#include "usart.h"

/**
 * @brief   can总线初始化，包括滤波器，中断等等
 */
void CAN_Init()
{
    CAN_FilterTypeDef can_filter_t;
    can_filter_t.FilterBank           = 0; //一个编号，can1用0，can2用14
    can_filter_t.FilterMode           = CAN_FILTERMODE_IDMASK;
    can_filter_t.FilterScale          = CAN_FILTERSCALE_32BIT;
    can_filter_t.FilterIdHigh         = 0x0000;
    can_filter_t.FilterIdLow          = 0x0000;
    can_filter_t.FilterMaskIdHigh     = 0x0000;
    can_filter_t.FilterMaskIdLow      = 0x0000;
    can_filter_t.FilterActivation     = ENABLE;
    can_filter_t.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter_t.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_t);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //启用 CAN1 的中断通知功能，当 FIFO0 有消息挂起时触发中断

    can_filter_t.FilterBank           = 14;
    can_filter_t.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_t);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); //启用 CAN1 的中断通知功能，当 FIFO0 有消息挂起时触发中断
}

void CanSendCmd(uint16_t id, uint8_t cmd[8])
{
    CAN_TxHeaderTypeDef tx_message;
    uint32_t            tx_mailbox;
    tx_message.StdId = id;
    tx_message.IDE   = CAN_ID_STD;
    tx_message.RTR   = CAN_RTR_DATA;
    tx_message.DLC   = 0x08;
    HAL_CAN_AddTxMessage(&hcan1, &tx_message, cmd, &tx_mailbox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef rx_message;
    uint8_t             rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_message, rx_data);
    uint32_t ID = rx_message.StdId;
    if (hcan->Instance == CAN2)
    {
        chassis.GetGimbalCmd(ID, rx_data);
    }

    if (hcan->Instance == CAN1)
    {
        chassis.ChassisCallback(ID, rx_data);
    }
}
