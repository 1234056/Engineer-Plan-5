//
// Created by mjw on 2022/10/7.
//
//shootc 增加模式
#include "shootc.h"
#include "drv_can.h"
#include "normal_pid.h"
#include "debugc.h"
#include "remotec.h"
#include "INS_task.h"
#include "math.h"
#include "gimbalc.h"
#include "usartio.h"
#include "usart.h"

shootc shoot;


/**
 * @brief 射击控制主循环，协调各子功能
 * @retval none
 */
void shootc::ControlLoop(void) //发弹主循环：模式切换和速率测试
{
	FricControl();
	SetRammer(); //拨弹轮速度设置
	Stuck_Check(); //卡弹检测
	Printf_Test();
}

/**
 * @brief 获取摩擦轮状态
 * @retval int8_t FRIC_ON(1): 摩擦轮已启动, FRIC_OFF(0): 摩擦轮未启动
 */
int8_t shootc::GetFricStatus(void)
{
	if (abs(speed_pids[1].getInput()) > 5000 && abs(speed_pids[2].getInput()) > 5000)
	{
		return FRIC_ON;
	}
	else return FRIC_OFF;
}

/**
 * @brief shootc 类构造函数，初始化 PID 参数
 */
shootc::shootc()
{
	ram_pos_pid.setComputeType(PositionPID_e);

	speed_pids[0].setWorkType(Ramp_e);
	speed_pids[0].setRampTargetTime(1);
	speed_pids[0].setRampTargetStep(120);
	speed_pids[0].setComputeType(IncrementPID_e);

	speed_pids[1].setWorkType(Normal_e);
	speed_pids[1].setRampTargetTime(3);
	speed_pids[1].setRampTargetStep(100);
	speed_pids[1].setComputeType(IncrementPID_e);

	speed_pids[2].setWorkType(Normal_e);
	speed_pids[2].setRampTargetTime(3);
	speed_pids[2].setRampTargetStep(100);
	speed_pids[2].setComputeType(IncrementPID_e);

	Channel_StopTimeMax = 50;
}

/**
 * @brief 摩擦轮控制函数
 * @retval none
 */
void shootc::FricControl(void)
{
	motors[1].update_angle();
	motors[2].update_angle();

	speed_pids[1].Update(motors[1].Motor_Speed);
	speed_pids[2].Update(motors[2].Motor_Speed);

	speed_pids[1].GetOutput();
	speed_pids[2].GetOutput();
}

/**
 * @brief 清除摩擦轮目标速度（直接归零）
 * @retval none
 */
void shootc::ShootSpeedClean(void) //clean不用斜坡
{
	speed_pids[1].setTarget(0);
	speed_pids[2].setTarget(0);

	if (abs(speed_pids[1].getInput()) < 1000 || abs(speed_pids[2].getInput()) < 1000) //清除漏电流，防止下载疯转
	{
		speed_pids[1].setOut(0);
		speed_pids[2].setOut(0);
	}
}

/**
 * @brief 清除拨弹轮目标速度（直接归零）
 * @retval none
 */
void shootc::RammerSpeedClean(void)
{
	speed_pids[0].setWorkType(Normal_e);
	speed_pids[0].setTarget(0);
}

/**
 * @brief 重置摩擦轮目标速度（使用斜坡）
 * @retval none
 */
void shootc::ShootSpeedReset()
{
	speed_pids[1].setWorkType(Ramp_e);
	speed_pids[1].setTarget(SHOOT_SPEED);

	speed_pids[2].setWorkType(Ramp_e);
	speed_pids[2].setTarget(-SHOOT_SPEED);
}

/**
 * @brief 设置拨弹轮目标速度
 * @param[in] Ram_Speed 目标速度值
 * @param[in] mode 设置模式:
 *                  1 = 连续发射模式
 *                  2 = 单发发射模式
 * @retval none
 */
void shootc::ShootSpeedTarget(float Ram_Speed, int8_t mode) //bug:有时候目标值弹跳 已解决
{
	ShootSpeedReset();
	if (mode == 1)
	{
		speed_pids[0].setWorkType(Ramp_e);
		speed_pids[0].setTarget(Ram_Speed);
		rammer_flag = 0;
	}
	else if (mode == 2)
	{
		if (rammer_flag == 0) //掉电初始时还是出现大幅度反转
		{
			ram_pos_pid.setErrAll(0);
			ram_pos_pid.setLastInput(0);
			speed_pids[0].setErrAll(0);
			motors[0].clear();
		}
		rammer_flag++;
		ram_pos_pid.setTarget(0 + rammer_flag * 45);
	}
}

/**
 * @brief 卡弹检测与处理
 * @retval none
 */
void shootc::Stuck_Check(void)
{
	int16_t Current = motors[0].Torque;
	// usart_printf("%d,%d\r\n", Current, stack_time);
	if (Current > 5000) //堵转
	{
		stack_time++;
	}
	if (stack_time >= 100) //堵转超过500ms
	{
		ram_pos_pid.setErrAll(0);
		ram_pos_pid.setErrAll(0);
		ShootSpeedTarget(-20, 1); //卡弹时以100rpm速度反转1s
		reverse_time++;
	}
	if (reverse_time >= reverse_time_max)
	{
		reverse_time = 0;
		stack_time = 0;
	}
}//堵转检测


int flag;

/**
 * @brief 设置拨弹轮控制逻辑（根据遥控器模式）
 * @retval none
 * @note 此函数包含 DR16 和 FSI6 两种遥控协议的实现
 */
void shootc::SetRammer(void)
{
#if defined(DT7)
	speed_pids[0].Update(motors[0].Motor_Speed);
	speed_pids[0].GetOutput();

	MyRemote.rc_ctrl.Get_Key().Q.portHandle();
	MyRemote.rc_ctrl.Get_Mouse().press_l.portHandle();
	int16_t RammerSpeed = 10;
	Channel_Max = 120.0f; //最大拨弹速度-遥控器

	switch (MyRemote.Control_Mode)
	{
	case KEY_MODE:
		{
			if ( permit && GetFricStatus() && motors[1].is_online && motors[2].is_online && omni.Get_fric_ram_status() == OPENRAMMER)
			{
				speed_pids[0].setWorkType(Ramp_e);
                speed_pids[0].setTarget(ram_pos_pid.getOut());
                if (rammer_flag == 0)
                {
                    speed_pids[0].setWorkType(Ramp_e);
                    speed_pids[0].setTarget(0);
                    ram_pos_pid.setErrAll(0); //积分项清除
                    ram_pos_pid.setTarget(0);
                }

				Channel_Now = MyRemote.rc_ctrl.Get_Mouse().press_l.Get_Now_State();
				if (MyRemote.rc_ctrl.Get_Mouse().press_l.Get_Is_Click_Once())
				{
					ShootSpeedTarget(RammerSpeed, 2);
				}

				motors[0].update_angle();
				ram_pos_pid.Update(motors[0].Angel_All);
				ram_pos_pid.GetOutput();

				if (Channel_Now == Channel_Last && Channel_Now != 0)
				{
					if (Channel_StopTime++ >= Channel_StopTimeMax) //速度环正常
					{
						RammerSpeed = 50;
						if (MyRemote.rc_ctrl.Get_Key().R.Get_Now_State() == 1) RammerSpeed = 120;//改成与当前热量相关
						ShootSpeedTarget(RammerSpeed, 1);
					}
				}
				else Channel_StopTime = 0;
				Channel_Last = Channel_Now;
			}
			else
			{
				RammerSpeedClean();
				ShootSpeedTarget(0, 1);
			}
			break;
	}
	case RC_MODE:
	{
		if (permit && GetFricStatus() && motors[1].is_online && motors[2].is_online && omni.Get_fric_ram_status() == OPENRAMMER)
		{
			speed_pids[0].setWorkType(Ramp_e);
            speed_pids[0].setTarget(ram_pos_pid.getOut());
            if (rammer_flag == 0)
            {
                speed_pids[0].setWorkType(Ramp_e);
                speed_pids[0].setTarget(0);
                ram_pos_pid.setTarget(0);
                ram_pos_pid.setErrAll(0); //积分项清除
            }

			Channel_Now = abs(MyRemote.rc_ctrl.Get_RC().ch[4] * Channel_Max / 660.0f);

			if (Channel_Now == Channel_Max && Channel_Last != Channel_Max)
			{
				ShootSpeedTarget(RammerSpeed, 2);
			}

			motors[0].update_angle();
			ram_pos_pid.Update(motors[0].Angel_All);
			ram_pos_pid.GetOutput();

			if (Channel_Now == Channel_Last && Channel_Now != 0)
			{
				if (Channel_StopTime++ >= Channel_StopTimeMax) //速度环正常 英雄：1000000  步兵：50
				{
					RammerSpeed = Channel_Now;
					ShootSpeedTarget(RammerSpeed, 1);
				}
			}
			else Channel_StopTime = 0;
			Channel_Last = Channel_Now;
			flag = FRIC_ON ;
		}
		else
		{
			RammerSpeedClean();
			speed_pids[0].setOut(0);
			ShootSpeedTarget(0, 1);
		}
		break;
	}
	}
	if(!motors[1].is_online && !motors[2].is_online || (flag== FRIC_ON && !GetFricStatus()))
	{
		ram_pos_pid.setTarget(0);
        ram_pos_pid.setErrAll(0);
        speed_pids[0].setErrAll(0);
        rammer_flag = 0;
        motors[0].clear();
		flag = FRIC_OFF;
	}
#elif defined(FSi6x)
	// motors[0].update_angle();
	speed_pids[0].Update(motors[0].Motor_Speed);
	speed_pids[0].GetOutput();
	int16_t RammerSpeed = 10;
	Channel_Max = 120.0f; //最大拨弹速度-遥控器
	if (permit && GetFricStatus() && motors[1].is_online && motors[2].is_online && omni.Get_fric_ram_status() == OPENRAMMER)
	{
		speed_pids[0].setWorkType(Ramp_e);
		speed_pids[0].setTarget(ram_pos_pid.getOut());
		if (rammer_flag == 0)
		{
			speed_pids[0].setWorkType(Ramp_e);
			speed_pids[0].setTarget(0);
			ram_pos_pid.setTarget(0);
			ram_pos_pid.setErrAll(0); //积分项清除

		}

		Channel_Now = MyRemote.getSwich_A();

		if (Channel_Now == 2 && Channel_Last != 2)
		{
			ShootSpeedTarget(RammerSpeed, 2);
		}

		motors[0].update_angle();
		ram_pos_pid.Update(motors[0].Angel_All);
		ram_pos_pid.GetOutput();

		if (Channel_Now == Channel_Last && Channel_Now == 2)
		{
			if (Channel_StopTime++ >= Channel_StopTimeMax) //速度环正常 英雄：1000000  步兵：50
			{
				RammerSpeed = 50;
				ShootSpeedTarget(RammerSpeed, 1);
			}
			usart_printf("%d\n",Channel_Now);
		}
		else Channel_StopTime = 0;
		Channel_Last = Channel_Now;
		flag = FRIC_ON ;
		if (Channel_Now!=2) {
			RammerSpeedClean();
			speed_pids[0].setOut(0);
			ShootSpeedTarget(0, 1);

		}


	}
	else
	{
		RammerSpeedClean();
		speed_pids[0].setOut(0);
		ShootSpeedTarget(0, 1);
	}
	if(!motors[1].is_online && !motors[2].is_online || (flag== FRIC_ON && !GetFricStatus()))
	{
		ram_pos_pid.setTarget(0);
		ram_pos_pid.setErrAll(0);
		speed_pids[0].setErrAll(0);
		rammer_flag = 0;
		motors[0].clear();
		flag = FRIC_OFF;
	}
#endif
}

/**
 * @brief 调试信息打印函数
 * @retval none
 */
void shootc::Printf_Test(void)
{
	//usart_printf("%d\n",Channel_Now);
	//放一些常用的打印
}