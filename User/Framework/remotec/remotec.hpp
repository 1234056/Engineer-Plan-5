//
// Created by Glucose_carbide on 25-8-8.
//

#ifndef REMOTEC_HPP
#define REMOTEC_HPP

#include <cctype>
#include "usartio.hpp"

typedef enum
{
    HIGH = 1,
    MIDDLE = 3,
    LOW = 2
} switch_status;

// TODO: 根据需要调整
// #define DR16
#define FSI6

#ifdef DR16

#define SBUS_RX_BUF_SIZE 255

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
#endif // DR16

#ifdef FSI6

#define RC_FRAME_LENGTH	25u
#define FS16_OFFSET	1024

#define FS16_CHANNEL_1      rc_right_horizontal
#define FS16_CHANNEL_2      rc_right_vertical
#define FS16_CHANNEL_3      rc_left_vertical
#define FS16_CHANNEL_4      rc_left_horizontal
#define FS16_CHANNEL_5      rc_knob_left
#define FS16_CHANNEL_6      rc_knob_right
#define FS16_CHANNEL_7      rc_switch_A
#define FS16_CHANNEL_8      rc_switch_B
#define FS16_CHANNEL_9      rc_switch_C
#define FS16_CHANNEL_10     rc_switch_D

#endif // FSI6

class cRemote : public cUsart
{
public:
    cRemote(UART_HandleTypeDef* local_huart, uint16_t buf_size, eUsartType type) :
        cUsart(local_huart, buf_size, type)
    {
    }
    [[nodiscard]] bool CheckOnline();
    void rxUserCallback() override;
    void UpdateRCTarget(float* rc_target,int16_t* mode) const;

private:
    void RcUpdateValue(const uint8_t* rc_data);

    int16_t rc_left_horizontal       = 0;
    int16_t rc_left_horizontal_last  = 0;
    int16_t rc_left_vertical         = 0;
    int16_t rc_left_vertical_last    = 0;
    int16_t rc_right_horizontal      = 0;
    int16_t rc_right_horizontal_last = 0;
    int16_t rc_right_vertical        = 0;
    int16_t rc_right_vertical_last   = 0;
    int16_t rc_switch_A              = HIGH;
    int16_t rc_switch_B              = HIGH;
    int16_t rc_switch_C              = MIDDLE;
    int16_t rc_switch_C_last         = MIDDLE;
    int16_t rc_switch_D              = HIGH;
    int16_t rc_knob_left             = 0;
    int16_t rc_knob_right            = 0;

    int16_t mouse_x = 0;
    int16_t mouse_y = 0;
    int16_t mouse_z = 0;
    uint8_t mouse_pressed_left = 0;
    uint8_t mouse_pressed_right = 0;

    uint16_t keyboard_values = 0;
    uint8_t pressed_W = 0;
    uint8_t pressed_S = 0;
    uint8_t pressed_A = 0;
    uint8_t pressed_D = 0;
    uint8_t pressed_Q = 0;
    uint8_t pressed_E = 0;
    uint8_t pressed_Shift = 0;
    uint8_t pressed_Ctrl = 0;

    int16_t counter = 1145;
    bool online = false;
    uint32_t times = 1919810;
};

#endif // REMOTEC_HPP
