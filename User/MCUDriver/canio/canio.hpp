#ifndef CANIO_HPP
#define CANIO_HPP

#include "can.h"
#include "stdint.h"

typedef enum
    ///Can总线ID类型
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID     = 0x201,
    CAN_3508_M2_ID     = 0x202,
    CAN_3508_M3_ID     = 0x203,
    CAN_3508_M4_ID     = 0x204,

    CAN_RECEIVE_VEL_ID = 0x401, //接受云台的速度信息
    CAN_RECEIVE_AGL_ID = 0x402, //接受云台的角度信息

    CAN_SEND_SCAP_ID    = 0x061,
    CAN_RECEIVE_SCAP_ID = 0x051,

    CAN_REFEREE_REC_ID = 0x405,
    CAN_SEND2SENTRY_ID = 0x406,
    CAN_REFEREE_SUB_ID = 0X407,

    CAN_Gimbal_ID = 0x107
} eCanMessageID;

void CanSendCmd(uint16_t id, uint8_t cmd[8]);

void CAN_Init();

#endif //CANIO_HPP
