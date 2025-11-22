//
// Created by ywr on 2025/10/23.
//

#ifndef ROBO_TEST_REMOTEC_H
#define ROBO_TEST_REMOTEC_H
#ifdef __cplusplus
extern "C" {
#endif
#include "gimbalc.h"
#include <cctype>
#include "Key_State.h"
#include "RC_Ctrl_t.h"


#include "remoteio.h"
#include "stm32f4xx_hal.h"
// #define DT7
#define FSi6x
class remotec;
void REMOTEC_Init(void);
#if defined(DT7)
#define SBUS_RX_BUF_NUM 36u

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
/* ----------------------- Internal Data ----------------------------------- */

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void REMOTEC_Init(void);

class remotec {
 public:
	//remote control data
	//遥控器控制变量
	RC_Ctrl_t rc_ctrl;
	int16_t RC_GetNewData = 0;//检测键值是否在发送/更新
	uint8_t Control_Mode = RC_MODE;
	bool is_online = 0;

	void RC_DataHandle();

	float yaw_speed;
	float pih_speed;
	float vx = 0;
	float vy = 0;
	int8_t CarMode;
	int8_t Last_CarMode = SUIDONG; //默认自由
	int8_t FricStatus ;
	int8_t Last_FricStatus = OPENFRIC;

	uint8_t portIsZimiao(void);
	uint8_t portIsRedrawing(void);

	void ShootSpeedTarget(float Shoot_Speed, float Ram_Speed, int8_t mode); //bug:有时候目标值弹跳 已解决
	void sbus_to_rc(volatile const uint8_t* sbus_buf);
	void update();
 private:
	void portSetYawSpeed(void);
	void portSetPihSpeed(void);
	void portSetVx(void);
	void portSetVy(void);
	void portSetCarMode(void);
	void portSetFric(void);
	void portSetHead(void);
	void Swich_ControlMode(void);
};
extern remotec MyRemote;


#elif defined(FSi6x)

#define SBUS_RX_BUF_NUM 50u
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


class remotec
{
public:
	bool is_online = 0; //遥控器是否在线
	uint32_t times_ = 0;
	float yaw_speed = 0;
	float pih_speed = 0;
	float vx = 0;
	float vy = 0;
	int8_t CarMode =SUIDONG;
	int8_t Last_CarMode = SUIDONG; //默认自由
	int8_t FricStatus = CLOSEFRIC;
	int8_t Last_FricStatus = OPENFRIC;
	uint8_t Control_Mode = RC_MODE;
	int16_t RC_GetNewData = 0;//检测键值是否在发送/更新
	int8_t ShootMode = NOSHOOT;

	void fsi6_sbus_to_rc(const uint8_t* rc_data);
	void RC_DataHandle();
	void portSetYawSpeed(void);
	void portSetPihSpeed(void);
	void portSetVx(void);
	void portSetVy(void);
	void portSetCarMode(void);
	void portSetFric(void);
	void portSetShootMode(void);
	void update(void);
	int getSwich_A(void){return rc_switch_A;}

	int16_t getrc_left_horizontal() const{return rc_left_horizontal;}
	int16_t getrc_left_vertical() const{return rc_left_vertical;}
	int16_t getrc_right_horizontal() const{return rc_right_horizontal;}



private:
	int16_t rc_left_horizontal       = 0;
	int16_t rc_left_horizontal_last  = 0;
	int16_t rc_left_vertical         = 0;
	int16_t rc_left_vertical_last    = 0;
	int16_t rc_right_horizontal      = 0;
	int16_t rc_right_horizontal_last = 0;
	int16_t rc_right_vertical        = 0;
	int16_t rc_right_vertical_last   = 0;
	int16_t rc_switch_A              = 1;
	int16_t rc_switch_B              = 1;
	int16_t rc_switch_C              = 1;
	int16_t rc_switch_C_last         = 1;
	int16_t rc_switch_A_Last         = 1;
	int16_t rc_switch_D              = 1;
	int16_t rc_knob_left             = 0;
	int16_t rc_knob_right            = 0;
};
extern remotec MyRemote;
#endif
#ifdef __cplusplus
	}
#endif
#endif

