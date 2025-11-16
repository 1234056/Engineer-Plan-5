//
// Created by Glucose_carbide on 25-8-8.
//

#include "remotec.hpp"
#include "main.h"
#include "usart.h"
#include "chassisc.hpp"

#ifdef DR16

/**
  * @brief          遥控器协议解析
  * @param[in]      rc_data: 原生数据指针
  * @retval         none
  */
void cRemote::RcUpdateValue(const uint8_t *rc_data)
{
    if (rc_data == nullptr)
        return;

    /* 遥控器上的 */
    rc_right_horizontal = static_cast<int16_t>(((rc_data[1] <<  8 | rc_data[0])                        & 0x7FF) - RC_CH_VALUE_OFFSET);
    rc_right_vertical   = static_cast<int16_t>(((rc_data[2] <<  5 | rc_data[1] >> 3)                   & 0x7FF) - RC_CH_VALUE_OFFSET);
    rc_left_horizontal  = static_cast<int16_t>(((rc_data[4] << 10 | rc_data[3] << 2 | rc_data[2] >> 6) & 0x7FF) - RC_CH_VALUE_OFFSET);
    rc_left_vertical    = static_cast<int16_t>(((rc_data[5] <<  7 | rc_data[4] >> 1)                   & 0x7FF) - RC_CH_VALUE_OFFSET);
    rc_switch_A = static_cast<int16_t>((rc_data[5] >> 4 & 0x0C) >> 2);
    rc_switch_D = static_cast<int16_t> (rc_data[5] >> 4 & 0x03);

    /* 鼠标上的(鼠标有z轴???) */
    mouse_x = static_cast<int16_t>(rc_data[7] << 8 | rc_data[6]);
    mouse_y = static_cast<int16_t>(rc_data[9] << 8 | rc_data[8]);
    mouse_z = static_cast<int16_t>(rc_data[11] << 8 | rc_data[10]);
    mouse_pressed_left = rc_data[12];
    mouse_pressed_right = rc_data[13];

    /* 键盘上的 */
    keyboard_values = rc_data[15] << 8 | rc_data[14];
    pressed_W		= static_cast<uint8_t>(keyboard_values & 0x01);
    pressed_S		= static_cast<uint8_t>(keyboard_values & 0x02);
    pressed_A		= static_cast<uint8_t>(keyboard_values & 0x04);
    pressed_D		= static_cast<uint8_t>(keyboard_values & 0x08);
    pressed_Q		= static_cast<uint8_t>(keyboard_values & 0x10);
    pressed_E		= static_cast<uint8_t>(keyboard_values & 0x20);
    pressed_Shift	= static_cast<uint8_t>(keyboard_values & 0x40);
    pressed_Ctrl	= static_cast<uint8_t>(keyboard_values & 0x80);
}

/**
 * @brief 检查遥控器是否在线
 * @return 是否在线
 */
bool cRemote::CheckOnline()
{
    times++;
    if (times > 100)
    {
        times = 1919810;
        online = false;
    }
    return online;
}

/**
 * @brief 串口中断回调
 */
void cRemote::rxUserCallback()
{
    times  = 0;
    online = true;
    RcUpdateValue(rx_buf);
}

/**
 * @brief          获取遥控器解算出的实际目标速度
 * @param rc_target 底盘目标速度数组[0]：x，[1]：y，[2]：z
 * @param mode 模式选择
 * @return 无
 */
void cRemote::UpdateRCTarget(float* rc_target, int16_t* mode) const
{
    rc_target[0] = static_cast<float>(rc_left_horizontal) / 6.6f;
    rc_target[1] = static_cast<float>(rc_left_vertical) / 6.6f;
    rc_target[2] = static_cast<float>(rc_right_horizontal) / 6.6f;
    *mode         = rc_switch_D;
}
#endif // DR16

#ifdef FSI6

/**
 *
 * @param   status :需要规范化的拨杆数据
 * @retval  与大疆DT7发送数据一致的拨杆数据(1:H,3:M,2:L)
 */
static int16_t ReturnSwitchStatus(int16_t status)
{
    switch (status)
    {
    case -784:
        return HIGH; // 1(这是一个枚举，对应的值就是1，下同)
    case 0:
        return MIDDLE; // 3
    case 783:
        return LOW; // 2
    default:
        return HIGH; // 1
    }
}

/**
 * @brief   FSi6x发送数据的解析
 * @param   rc_data :接收数据指针
 * @retval  None
 */
void cRemote::RcUpdateValue(const uint8_t* rc_data)
{
    if (rc_data[0] != 0x0F || rc_data[24] != 0x00)                //判断数据正确与否
        return;

    FS16_CHANNEL_1  = static_cast<int16_t>(((rc_data[ 2] <<  8 | rc_data[ 1])                        & 0x7FF) - FS16_OFFSET);
    FS16_CHANNEL_2  = static_cast<int16_t>(((rc_data[ 3] <<  5 | rc_data[ 2] >> 3)                   & 0x7FF) - FS16_OFFSET);
    FS16_CHANNEL_3  = static_cast<int16_t>(((rc_data[ 5] << 10 | rc_data[ 4] << 2 | rc_data[3] >> 6) & 0x7FF) - FS16_OFFSET);
    FS16_CHANNEL_4  = static_cast<int16_t>(((rc_data[ 6] <<  7 | rc_data[ 5] >> 1)                   & 0x7FF) - FS16_OFFSET);
    FS16_CHANNEL_5  = static_cast<int16_t>(((rc_data[ 7] <<  4 | rc_data[ 6] >> 4)                   & 0x7FF) - FS16_OFFSET);
    FS16_CHANNEL_6  = static_cast<int16_t>(((rc_data[ 9] <<  9 | rc_data[ 8] << 1 | rc_data[7] >> 7) & 0x7FF) - FS16_OFFSET);
    FS16_CHANNEL_7  = static_cast<int16_t>(((rc_data[10] <<  6 | rc_data[ 9] >> 2)                   & 0x7FF) - FS16_OFFSET);
    FS16_CHANNEL_8  = static_cast<int16_t>(((rc_data[11] <<  3 | rc_data[10] >> 5)                   & 0x7FF) - FS16_OFFSET);
    FS16_CHANNEL_9  = static_cast<int16_t>(((rc_data[13] <<  8 | rc_data[12])                        & 0x7FF) - FS16_OFFSET);
    FS16_CHANNEL_10 = static_cast<int16_t>(((rc_data[14] <<  5 | rc_data[13] >> 3)                   & 0x7FF) - FS16_OFFSET);

    rc_switch_A = ReturnSwitchStatus(rc_switch_A);
    rc_switch_B = ReturnSwitchStatus(rc_switch_B);
    rc_switch_C = ReturnSwitchStatus(rc_switch_C);
    rc_switch_D = ReturnSwitchStatus(rc_switch_D);

    if (rc_left_horizontal == rc_left_horizontal_last &&
        rc_left_vertical == rc_left_vertical_last &&
        rc_right_horizontal == rc_right_horizontal_last &&
        rc_right_vertical == rc_right_vertical_last &&
        rc_switch_C == rc_switch_C_last)
        counter++;
    else
    {
        counter = 0;
        online = true;
    }

    if (counter >= 500 || rc_switch_C == HIGH)
    {
        counter = 1145;
        online = false;
    }

    rc_left_horizontal_last  = rc_left_horizontal;
    rc_left_vertical_last    = rc_left_vertical;
    rc_right_horizontal_last = rc_right_horizontal;
    rc_right_vertical_last   = rc_right_vertical;
    rc_switch_C_last         = rc_switch_C;
}

/**
 * @brief 串口中断回调
 */
void cRemote::rxUserCallback()
{
    RcUpdateValue(rx_buf);
}

[[nodiscard]] bool cRemote::CheckOnline()
{
    return online;
}

/*
 * @brief          获取遥控器解算出的实际目标速度
 * @param rc_target 底盘目标速度数组[0]：x，[1]：y，[2]：z
 * @return 无
 */
void cRemote::UpdateRCTarget(float* rc_target, int16_t* mode) const
{
    rc_target[0] = static_cast<float>(rc_left_vertical) / (7.84f * 4);
    rc_target[1] = -static_cast<float>(rc_left_horizontal) / (7.84f * 4);
    rc_target[2] = -static_cast<float>(rc_right_horizontal);
    *mode = rc_switch_D;
}

#endif
