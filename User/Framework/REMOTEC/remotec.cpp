//
// Created by ywr on 2025/10/23.
//

#include "remotec.h"
#include "usart.h"
#include "debugc.h"
#include "stdlib.h"
#include "cmath"
#include "cstring"
#include "cstdio"
#include "INS_task.h"
#include "gimbalc.h"
#include "normal_pid.h"
#include "shootc.h"
#include "drv_can.h"
#include "ITtask.h"

remotec MyRemote;

static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
#if defined(DT7)

/**
 * @brief 初始化遥控器硬件接口
 * @param[in] none
 * @retval none
 */
void REMOTEC_Init(void)
{
	REMOTEIO_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

/**
 * @brief USART3 中断处理函数，处理遥控器数据接收
 * @param[in] none
 * @retval none
 */
void REMOTEC_UartIrqHandler(void)
{
	if (huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
	{
		__HAL_UART_CLEAR_PEFLAG(&huart3);
	}
	else if (USART3->SR & UART_FLAG_IDLE)
	{
		static uint16_t this_time_rx_len = 0;

		__HAL_UART_CLEAR_PEFLAG(&huart3);

		if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			/* Current memory buffer used is Memory 0 */

			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart3_rx);

			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

			//reset set_data_length
			//重新设定数据长度
			hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

			//set memory buffer 1
			//设定缓冲区1
			hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

			//enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart3_rx);

			if (this_time_rx_len == RC_FRAME_LENGTH)
			{
				MyRemote.sbus_to_rc(sbus_rx_buf[0]);
			}
		}
		else
		{
			/* Current memory buffer used is Memory 1 */
			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart3_rx);

			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

			//reset set_data_lenght
			//重新设定数据长度
			hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

			//set memory buffer 0
			//设定缓冲区0
			DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

			//enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart3_rx);

			if (this_time_rx_len == RC_FRAME_LENGTH)
			{
				//处理遥控器数据
				MyRemote.sbus_to_rc(sbus_rx_buf[1]);
			}
		}
		MyRemote.RC_DataHandle();
	}
}


/**
  * @brief          DR16遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指针
  * @retval         none
  */

void remotec::sbus_to_rc(volatile const uint8_t* sbus_buf)
{
	if (sbus_buf == NULL || &rc_ctrl == NULL)
	{
		return;
	}
	rc_ctrl.Get_RC().ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
	rc_ctrl.Get_RC().ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
	rc_ctrl.Get_RC().ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
			(sbus_buf[4] << 10)) & 0x07ff;
	rc_ctrl.Get_RC().ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
	rc_ctrl.Get_RC().s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
	rc_ctrl.Get_RC().s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
	rc_ctrl.Get_Mouse().x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
	rc_ctrl.Get_Mouse().y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
	rc_ctrl.Get_Mouse().z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis 滚轮
	rc_ctrl.Get_Mouse().press_l.Set_Now_State(sbus_buf[12]);                                  //!< Mouse Left Is Press ?
	rc_ctrl.Get_Mouse().press_r.Set_Now_State(sbus_buf[13]);                                  //!< Mouse Right Is Press ?
	rc_ctrl.Get_Key().value =
			sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value  W S A D Q E SHIFT CTRL
	rc_ctrl.Get_Key().W = (rc_ctrl.Get_Key().value & 0x01);
	rc_ctrl.Get_Key().S = (rc_ctrl.Get_Key().value & 0x02) >> 1;
	rc_ctrl.Get_Key().A = (rc_ctrl.Get_Key().value & 0x04) >> 2;
	rc_ctrl.Get_Key().D = (rc_ctrl.Get_Key().value & 0x08) >> 3;
	rc_ctrl.Get_Key().SHIFT.Set_Now_State((rc_ctrl.Get_Key().value & 0x10) >> 4);
	rc_ctrl.Get_Key().CONTRL.Set_Now_State((rc_ctrl.Get_Key().value & 0x20) >> 5);
	rc_ctrl.Get_Key().Q.Set_Now_State((rc_ctrl.Get_Key().value & 0x40) >> 6);
	rc_ctrl.Get_Key().E.Set_Now_State((rc_ctrl.Get_Key().value & 0x80) >> 7);
	rc_ctrl.Get_Key().R.Set_Now_State((rc_ctrl.Get_Key().value & 0x100) >> 8);
	rc_ctrl.Get_Key().F.Set_Now_State((rc_ctrl.Get_Key().value & 0x200) >> 9);
	rc_ctrl.Get_Key().G.Set_Now_State((rc_ctrl.Get_Key().value & 0x400) >> 10);
	rc_ctrl.Get_Key().Z.Set_Now_State((rc_ctrl.Get_Key().value & 0x800) >> 11);
	rc_ctrl.Get_Key().X.Set_Now_State((rc_ctrl.Get_Key().value & 0x1000) >> 12);
	rc_ctrl.Get_Key().C.Set_Now_State((rc_ctrl.Get_Key().value & 0x2000) >> 13);
	rc_ctrl.Get_Key().V.Set_Now_State((rc_ctrl.Get_Key().value & 0x4000) >> 14);
	rc_ctrl.Get_Key().B.Set_Now_State((rc_ctrl.Get_Key().value & 0x8000) >> 15);
	rc_ctrl.Get_RC().ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //拨盘

	rc_ctrl.Get_RC().ch[0] -= RC_CH_VALUE_OFFSET;
	rc_ctrl.Get_RC().ch[1] -= RC_CH_VALUE_OFFSET;
	rc_ctrl.Get_RC().ch[2] -= RC_CH_VALUE_OFFSET;
	rc_ctrl.Get_RC().ch[3] -= RC_CH_VALUE_OFFSET;
	rc_ctrl.Get_RC().ch[4] -= RC_CH_VALUE_OFFSET;
	RC_GetNewData = 0;
}

/**
 * @brief 遥控器数据处理，包括摇杆死区处理和异常值检测
 * @param[in] none
 * @retval none
 */
void remotec::RC_DataHandle()
{
	if (abs(rc_ctrl.Get_RC().ch[0]) < 5)rc_ctrl.Get_RC().ch[0] = 0;
	if (abs(rc_ctrl.Get_RC().ch[1]) < 5)rc_ctrl.Get_RC().ch[1] = 0;
	if (abs(rc_ctrl.Get_RC().ch[2]) < 5)rc_ctrl.Get_RC().ch[2] = 0;
	if (abs(rc_ctrl.Get_RC().ch[3]) < 5)rc_ctrl.Get_RC().ch[3] = 0;
	if (abs(rc_ctrl.Get_RC().ch[4]) < 5)rc_ctrl.Get_RC().ch[4] = 0;
	if (abs(rc_ctrl.Get_RC().ch[0]) > 670 || abs(rc_ctrl.Get_RC().ch[3]) > 670)
	{
		memset(&rc_ctrl, 0, sizeof(RC_Ctrl_t)); //异常值
	}
}

/**
 * @brief 设置云台偏航轴目标速度
 * @param[in] none
 * @retval none
 */
void remotec::portSetYawSpeed(void)
{
	switch (Control_Mode)
	{
	case KEY_MODE:
		if (abs(rc_ctrl.Get_Mouse().x) < 50)
			yaw_speed = -rc_ctrl.Get_Mouse().x * 0.5; //这儿后期加等级分档位
		else
			yaw_speed = -rc_ctrl.Get_Mouse().x * 2.0; //这儿后期加等级分档位
		break;
	case RC_MODE:
		yaw_speed = -rc_ctrl.Get_RC().ch[0] * 360 / 660.0f;
		break;
	}
}

/**
 * @brief 设置云台俯仰轴目标速度
 * @param[in] none
 * @retval none
 */
void remotec::portSetPihSpeed(void)
{
	switch (Control_Mode)
	{
	case KEY_MODE:
		if (abs(rc_ctrl.Get_Mouse().y) < 50)
			pih_speed = -rc_ctrl.Get_Mouse().y * 0.5;
		else
			pih_speed = -rc_ctrl.Get_Mouse().y * 1.5;
		break;
	case RC_MODE:
		pih_speed = rc_ctrl.Get_RC().ch[1] * 360 / 660.0f;
		break;
	}
}

/**
 * @brief 设置底盘 X 轴速度
 * @param[in] none
 * @retval none
 */
void remotec::portSetVx(void)
{
	switch (Control_Mode)
	{
	case KEY_MODE:
		vx = (rc_ctrl.Get_Key().D - rc_ctrl.Get_Key().A) * 80.0; //这儿后期加等级分档位
		if (rc_ctrl.Get_Key().SHIFT.Get_Now_State() == 1)
			vx *= 2.4;
		break;
	case RC_MODE:
		vx = rc_ctrl.Get_RC().ch[2] * 200.0f / 660.0f;
		break;
	}
}

/**
 * @brief 设置底盘 Y 轴速度
 * @param[in] none
 * @retval none
 */
void remotec::portSetVy(void)
{
	switch (Control_Mode)
	{
	case KEY_MODE:
		vy = (rc_ctrl.Get_Key().W - rc_ctrl.Get_Key().S) * 80.0;
		if (rc_ctrl.Get_Key().SHIFT.Get_Now_State() == 1)
			vy *= 2.4;
		break;
	case RC_MODE:
		vy = rc_ctrl.Get_RC().ch[3] * 200.0f / 660.0f;
		break;
	}
}

/**
 * @brief 设置底盘运动模式
 * @param[in] none
 * @retval none
 */
void remotec::portSetCarMode(void)
{
	rc_ctrl.Get_Key().V.portHandle();
	rc_ctrl.Get_Key().F.portHandle();
	switch (Control_Mode)
	{
	case KEY_MODE:
		if (rc_ctrl.Get_Key().V.Get_Is_Click_Once() && Last_CarMode == SUIDONG)
		{
			CarMode = TUOLUO;
		}
		else if (rc_ctrl.Get_Key().V.Get_Is_Click_Once() && Last_CarMode == TUOLUO)
		{
			CarMode = SUIDONG;
		}
		else CarMode = Last_CarMode;

		break;
	case RC_MODE:
		CarMode = rc_ctrl.Get_RC().s[0];
		break;
	}
	Last_CarMode = CarMode;
}

/**
 * @brief 设置摩擦轮状态
 * @param[in] none
 * @retval none
 */
void remotec::portSetFric(void)
{
	rc_ctrl.Get_Key().G.portHandle();
	rc_ctrl.Get_Key().F.portHandle();
	switch (Control_Mode)
	{
	case KEY_MODE:
		if (rc_ctrl.Get_Key().G.Get_Is_Click_Once() && Last_FricStatus != OPENFRIC)
		{
			FricStatus = OPENFRIC;
		}
		else if (rc_ctrl.Get_Key().G.Get_Is_Click_Once() && Last_FricStatus != CLOSEFRIC)
		{
			FricStatus = CLOSEFRIC;
		}
		else FricStatus = Last_FricStatus;

		break;
	case RC_MODE:
		FricStatus = rc_ctrl.Get_RC().s[1];
		break;
	}
	Last_FricStatus = FricStatus;
}

/**
 * @brief 检查是否正在补弹
 * @param[in] none
 * @retval uint8_t 1: 补弹中, 0: 非补弹状态
 */
uint8_t remotec::portIsRedrawing(void)
{
	rc_ctrl.Get_Key().B.portHandle();
	return rc_ctrl.Get_Key().B.Get_Now_State();
}

/**
 * @brief 设置云台朝向目标角度
 * @param[in] none
 * @retval none
 */
void remotec::portSetHead(void)
{
	int16_t Target = Get_ChassisTarget();
	rc_ctrl.Get_Key().X.portHandle();
	rc_ctrl.Get_Key().Q.portHandle();
	rc_ctrl.Get_Key().E.portHandle();
	if (rc_ctrl.Get_Key().Q.Get_Is_Click_Once())
	{
		Set_ChassisTarget(Target + 45);
	}

	if (rc_ctrl.Get_Key().E.Get_Is_Click_Once())
	{
		Set_ChassisTarget(Target - 45);
	}

	if (rc_ctrl.Get_Key().X.Get_Is_Click_Once())
	{
		Set_YawTarget(180);
		Set_ChassisTarget(Target + 180);
	}
}

/**
 * @brief 切换遥控器控制模式（键鼠/遥控器）
 * @param[in] none
 * @retval none
 */
void remotec::Swich_ControlMode(void)
{
	switch (rc_ctrl.Get_RC().s[1])
	{
	case TOKEY:
		Control_Mode = KEY_MODE;
		break;
	default:
		Control_Mode = RC_MODE;
		break;
	}
}

/**
 * @brief 更新遥控器所有状态
 * @param[in] none
 * @retval none
 */
void remotec::update()
{
	RC_GetNewData++;
	if (RC_GetNewData > 50)
	{
		is_online = 0;
	}
	else is_online = 1;
	// 复位
	if (RC_GetNewData >= 1000) {
		RC_GetNewData = 51;
	}

	Swich_ControlMode();
	portSetFric();
	portSetCarMode();
	portSetVx();
	portSetVy();
	portSetPihSpeed();
	portSetYawSpeed();
	portSetHead();
}



#elif defined(FSi6x)
enum switch_e
{
	HIGH,
	MIDDLE,
	LOW
};

/**
 * @brief 转换开关状态值为枚举值
 * @param[in] status 原始开关状态值
 * @retval switch_e 转换后的开关状态枚举值
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
 * @brief 初始化遥控器硬件接口
 * @param[in] none
 * @retval none
 */
void REMOTEC_Init(void)
{
	REMOTEIO_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

/**
 * @brief USART3 中断处理函数，处理遥控器数据接收
 * @param[in] none
 * @retval none
 */
void REMOTEC_UartIrqHandler(void)
{
	if (huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
	{
		__HAL_UART_CLEAR_PEFLAG(&huart3);
	}
	else if (USART3->SR & UART_FLAG_IDLE)
	{
		static uint16_t this_time_rx_len = 0;

		__HAL_UART_CLEAR_PEFLAG(&huart3);

		if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			/* Current memory buffer used is Memory 0 */

			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart3_rx);

			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

			//reset set_data_length
			//重新设定数据长度
			hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

			//set memory buffer 1
			//设定缓冲区1
			hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

			//enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart3_rx);

			if (this_time_rx_len == RC_FRAME_LENGTH)
			{
				MyRemote.fsi6_sbus_to_rc(sbus_rx_buf[0]);
			}
		}
		else
		{
			/* Current memory buffer used is Memory 1 */
			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart3_rx);

			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

			//reset set_data_lenght
			//重新设定数据长度
			hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

			//set memory buffer 0
			//设定缓冲区0
			DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

			//enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart3_rx);

			if (this_time_rx_len == RC_FRAME_LENGTH)
			{
				//处理遥控器数据
				MyRemote.fsi6_sbus_to_rc(sbus_rx_buf[1]);
			}
		}
		MyRemote.RC_DataHandle();
	}
}

/**
 * @brief 将 FSI6 原始数据解析为遥控器控制结构
 * @param[in] rc_data FSI6 原始数据指针
 * @retval none
 */
void remotec::fsi6_sbus_to_rc(const uint8_t* rc_data)
{
    if (rc_data[0] != 0x0F || rc_data[24] != 0x00)
        return;

    FS16_CHANNEL_1 = static_cast<int16_t>(((rc_data[2] << 8 | rc_data[1]) & 0x7FF) - FS16_OFFSET);
    FS16_CHANNEL_2 = static_cast<int16_t>(((rc_data[3] << 5 | rc_data[2] >> 3) & 0x7FF) - FS16_OFFSET);
    FS16_CHANNEL_3 = static_cast<int16_t>(((rc_data[5] << 10 | rc_data[4] << 2 | rc_data[3] >> 6) & 0x7FF) -
        FS16_OFFSET);
    FS16_CHANNEL_4 = static_cast<int16_t>(((rc_data[6] << 7 | rc_data[5] >> 1) & 0x7FF) - FS16_OFFSET);
    FS16_CHANNEL_5 = static_cast<int16_t>(((rc_data[7] << 4 | rc_data[6] >> 4) & 0x7FF) - FS16_OFFSET);
    FS16_CHANNEL_6 = static_cast<int16_t>(((rc_data[9] << 9 | rc_data[8] << 1 | rc_data[7] >> 7) & 0x7FF) -
        FS16_OFFSET);
    FS16_CHANNEL_7  = static_cast<int16_t>(((rc_data[10] << 6 | rc_data[9] >> 2) & 0x7FF) - FS16_OFFSET);
    FS16_CHANNEL_8  = static_cast<int16_t>(((rc_data[11] << 3 | rc_data[10] >> 5) & 0x7FF) - FS16_OFFSET);
    FS16_CHANNEL_9  = static_cast<int16_t>(((rc_data[13] << 8 | rc_data[12]) & 0x7FF) - FS16_OFFSET);
    FS16_CHANNEL_10 = static_cast<int16_t>(((rc_data[14] << 5 | rc_data[13] >> 3) & 0x7FF) - FS16_OFFSET);

    rc_switch_A = ReturnSwitchStatus(rc_switch_A);
    rc_switch_B = ReturnSwitchStatus(rc_switch_B);
    rc_switch_C = ReturnSwitchStatus(rc_switch_C);
    rc_switch_D = ReturnSwitchStatus(rc_switch_D);

	if (rc_switch_C == HIGH || rc_data[23] >> 2 & 0x01 || rc_data[23] >> 3 & 0x01)
	{
		RC_GetNewData = 1145;
		is_online = false;
	}
	else
	{
		RC_GetNewData = 0;
		is_online = true;
	}

    rc_left_horizontal_last  = rc_left_horizontal;
    rc_left_vertical_last    = rc_left_vertical;
    rc_right_horizontal_last = rc_right_horizontal;
    rc_right_vertical_last   = rc_right_vertical;
    rc_switch_C_last         = rc_switch_C;
	rc_switch_A_Last         = rc_switch_A;
}


/**
 * @brief 遥控器异常数据处理
 * @param[in] none
 * @retval none
 */
void remotec::RC_DataHandle()
{
	if (abs(rc_right_horizontal) < 5)rc_right_horizontal = 0;
	if (abs(rc_left_horizontal) < 5)rc_left_horizontal = 0;
	if (abs(rc_right_horizontal) < 5)rc_right_horizontal = 0;
	if (abs(rc_right_vertical) < 5)rc_right_vertical = 0;
}

/**
 * @brief 设置云台偏航轴目标速度
 * @param[in] none
 * @retval none
 */
void remotec::portSetYawSpeed(void)
{
	yaw_speed = -(float)rc_right_horizontal * 360 / 784.0f;
}

/**
 * @brief 设置云台俯仰轴目标速度
 * @param[in] none
 * @retval none
 */
void remotec::portSetPihSpeed(void)
{
	pih_speed = (float)rc_right_vertical * 360 / 784.0f;
}

/**
 * @brief 设置底盘 X 轴速度
 * @param[in] none
 * @retval none
 */
void remotec::portSetVx(void)
{
	vx = (float)rc_left_horizontal * 200.0f / 784.0f;
}

/**
 * @brief 设置底盘 Y 轴速度
 * @param[in] none
 * @retval none
 */
void remotec::portSetVy(void)
{
	vy = (float)rc_left_vertical * 200.0f / 784.0f;
}

/**
 * @brief 设置底盘运动模式
 * @param[in] none
 * @retval none
 */
void remotec::portSetCarMode(void)
{
	if (rc_switch_C == MIDDLE)
	{
		CarMode = SUIDONG;
	}
	else if (rc_switch_C == LOW)
	{
		CarMode = TUOLUO;
	}
	Last_CarMode = CarMode;
}

/**
 * @brief 设置摩擦轮状态
 * @param[in] none
 * @retval none
 */
void remotec::portSetFric(void)
{
	if (rc_switch_D == HIGH)
	{
		FricStatus = CLOSEFRIC;
	}
	else if (rc_switch_D == LOW)
	{
		FricStatus = OPENFRIC;
	}
	Last_FricStatus = FricStatus;
}

/**
 * @brief 设置发射模式
 * @param[in] none
 * @retval none
 */
void remotec::portSetShootMode(void)
{
	if (rc_switch_A == HIGH && rc_switch_A_Last == HIGH)
	{
		ShootMode = NOSHOOT;
	}
	if (rc_switch_A == LOW && rc_switch_A_Last == HIGH)
	{
		ShootMode = SINGLESHOOT;
	}
	if (rc_switch_A == LOW && rc_switch_A_Last == LOW)
	{
		ShootMode = CONTINUESHOOT;
	}
}

/**
 * @brief 更新遥控器所有状态
 * @param[in] none
 * @retval none
 */
void remotec::update()
{
	portSetFric();
	portSetCarMode();
	portSetVx();
	portSetVy();
	portSetPihSpeed();
	portSetYawSpeed();
}
#endif