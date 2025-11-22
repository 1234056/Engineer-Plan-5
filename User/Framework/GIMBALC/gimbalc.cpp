//
// Created by mjw on 2022/9/19.
//
#include "gimbalc.h"
#include "drv_can.h"
#include "normal_pid.h"
#include "debugc.h"
#include "remotec.h"
#include "INS_task.h"
#include <cmath>
#include "shootc.h"
// #include "kalman.h"
#include "iwdgc.h"
#include "slidingmodec.h"
#include <complex.h>
#include <remotec.h>
#include "usartio.h"
gimbalc omni;

/**
 * @brief 云台控制类的构造函数
 *
 * 初始化PID控制器、控制参数、云台偏航角等。设置位置控制和速度控制的PID参数。
 */
gimbalc::gimbalc()
{
	//matlab pid
	PID_initialize();

	YawBias = ChassisYawTarget;

	Pid_In.PihP_MO = 300;
	Pid_In.PihS_MO = 20192;
	Pid_In.YawP_MO = 300;
	Pid_In.YawS_MO = 30192;
	Pid_In.YawP_LO = -Pid_In.YawP_MO;
	Pid_In.YawS_LO = -Pid_In.YawS_MO;
	Pid_In.PihP_LO = -Pid_In.PihP_MO;
	Pid_In.PihS_LO = -Pid_In.PihS_MO;

	pos_pid[2].setOutStep(5000);
	pos_pid[2].setOutMax(300);
	pos_pid[2].setErrAllMax(30);

	speed_pid[2].setOutMax(30192);
	speed_pid[2].setErrAllMax(7192);
}

/**
 * @brief 选择控制模式
 *
 * 根据传入的模式（TOKEY_MODE、GYR_MODE、ZIMIAO_MODE），设置不同的PID参数和滑模控制参数。
 *
 * @param mode 控制模式
 */
void gimbalc::CarChoose(int8_t mode)
{
	switch (mode)
	{
	case TOKEY_MODE:
		{
			YawSMC.setC(28);
			YawSMC.setK(150);

			Pid_In.PihP_P = 1.2;//云台pih轴位置环
			Pid_In.PihP_I = 0;
			Pid_In.PihP_D = 0.15;
			Pid_In.PihP_N = 75;
			Pid_In.Pih_Dif_Gain = 0.12; //前馈一阶差分增益

			Pid_In.PihS_P = 600;//pih 300
			Pid_In.PihS_I = 750;
			Pid_In.PihS_D = 0.001;
			Pid_In.PihS_N = 20;

			break;
		}

	case TORC_MODE:
		{
			YawSMC.setC(35);
			YawSMC.setK(160);

			Pid_In.PihP_P = 2.3;//云台pih轴位置环
			Pid_In.PihP_I = 0;
			Pid_In.PihP_D = 0.12;
			Pid_In.PihP_N = 75;
			Pid_In.Pih_Dif_Gain = 0.12; //前馈一阶差分增益

			Pid_In.PihS_P = 600;//pih 300
			Pid_In.PihS_I = 600;
			Pid_In.PihS_D = 0.001;
			Pid_In.PihS_N = 20;
			break;
		}
	}
}

/**
 * @brief 计算控制算法
 *
 * 更新云台电机角度、执行PID控制、执行滑模控制和Matlab PID控制等。
 */
void gimbalc::AlgorithmCompute()
{

	motors[0].update_angle();
	motors[1].update_angle();

	//普通pid
	ChassisYawPid.Update(motors[0].Angel_Ecd);
	pos_pid[0].Update(motors[0].Angel_All);
	pos_pid[1].Update(motors[1].Angel_All);

	ChassisYawPid.GetOutput();
	pos_pid[0].GetOutput();
	pos_pid[1].GetOutput();

	speed_pid[0].setTarget(pos_pid[0].getOut());
	speed_pid[1].setTarget(pos_pid[1].getOut());

	speed_pid[0].Update(motors[0].Motor_Speed);
	speed_pid[1].Update(motors[1].Motor_Speed);

	speed_pid[0].GetOutput();
	speed_pid[1].GetOutput();

	//滑模控制
	YawSMC.SMC_Tick(motors[0].Angel_All, motors[0].Motor_Speed * 6);

	//Matlab的PID
	Pid_In.YawAngle_Now = motors[0].Angel_All;
	Pid_In.YawSpeed_Now = motors[0].Motor_Speed;
	Pid_In.PihAngle_Now = motors[1].Angel_All;
	Pid_In.PihSpeed_Now = motors[1].Motor_Speed;
	PID_step(1);

	//底盘随动前馈
	ChassisYawPid.setOut(ChassisYawPid.getOut() - Chassis_DifGain * (pos_pid[0].getTarget() - pos_pid[0].getLastTarget()));
	pos_pid[0].setLastTarget(pos_pid[0].getTarget());

}

/**
 * @brief 限制pih角度
 *
 * 根据当前模式（ECD_MODE 或 GYR_MODE）限制pih角度的上下限。
 */
void gimbalc::Pih_Limit()
{
	if ( motors[1].Which_Mode == ECD_MODE )
	{
		if (PihTarget > Pih_EcdUpLimit) //ecd pitch限位
		{
			PihTarget = Pih_EcdUpLimit;
		}
		if (PihTarget  < Pih_EcdLowLimit)
		{
			PihTarget  = Pih_EcdLowLimit;
		}
	}
	else
	{
		if (PihTarget > Pih_GyrUpLimit) //gyr pitch限位
		{
			PihTarget = Pih_GyrUpLimit;
		}
		if (PihTarget  < Pih_GyrLowLimit)
		{
			PihTarget  = Pih_GyrLowLimit;//-30
		}
	}
}

/**
 * @brief 重置yaw角度
 *
 * 清除yaw电机的角度，重新初始化yaw角度和PID误差。
 */
void gimbalc::Yaw_EcdClean(void) //无问题
{
	motors[0].MotorAngel_ALL = 0;
	motors[0].NowAngel = 0;
	motors[0].Angel_Ecd = motors[0].Angel * 360.0f / (8192.0f);
	ChassisYawPid.setErrAll(0);
} //ecd重置

/**
 * @brief 保护模式
 *
 * 检查电机是否在线，IMU是否正常，遥控器是否在线，并根据情况设置保护模式。
 */
void gimbalc::Protect_Mode()
{
	//检查电机、IMU、遥控器的连接状态，如果有任一设备离线，则进入OFFLINE保护模式
	if(!motors[0].is_online || !motors[1].is_online || IS_IMU_OK == 0 || !MyRemote.is_online)
	{
		Protect_flag = ONLINE;
	}
	else
	{
		Protect_flag = ONLINE;
	}

	if (Protect_flag == ONLINE)
	{	// 如果上次警告不是TOKEY且当前警告是TOKEY，进入摩擦轮开启保护模式
		if(Last_Warning != TOKEY && warning == TOKEY)
		{
			fric_ram_status = OPENRAMMER;
			warning = OPENFRIC;
			MyRemote.Last_FricStatus = OPENFRIC;
		}
		//关闭摩擦轮	保护模式
		switch (warning)
		{
		case CLOSEFRIC:
		{
			fric_ram_status = CLOSERAMMER;
			shoot.ShootSpeedClean();
			break;
		}

		case OPENFRIC:
		{
			if (Last_Warning != OPENFRIC || Last_ProtectFlag == OFFLINE) //不能在断电的时候由连发切单发，且还是有一定幅度的反转 大概10°
			{
				//rammer电机重置
				shoot.speed_pids[0].setErrAll(0);
                shoot.ram_pos_pid.setTarget(0);
                shoot.ram_pos_pid.setErrAll(0);
				shoot.setrammerflag(0);
				shoot.SetRammer();//通电时默认连发
				shoot.motors[0].clear();
			}
			shoot.ShootSpeedReset();
			fric_ram_status = OPENRAMMER;
			break;
		}
		}
	}
	else if (Protect_flag == OFFLINE)
	{
		Pid_Out.YawCurrent = 0;
		Pid_Out.PihCurrent = 0;
		speed_pid[0].setTarget(0);
        speed_pid[1].setTarget(0);
		YawSMC.setU(0);

		shoot.ShootSpeedClean();
		shoot.RammerSpeedClean();
		shoot.speed_pids[0].setOut(0);

		TargetInit();
		can.YawSendCurrent(0);
		can.PitchSendCurrent(0);

		fric_ram_status = CLOSERAMMER;
		CarMode = SUIDONG;
		Last_CarMode = SUIDONG;
		ChassisYawPid.setErrAll(0);
	}
	Last_Warning = warning;
	Last_ProtectFlag = Protect_flag;
}

/**
 * @brief 底盘通讯循环
 *
 * 根据遥控器的状态更新底盘控制指令，处理不同的状态。
 */
void gimbalc::ChassisComLoop()
{
	is_online = 0xff;
	if (Protect_flag == OFFLINE)
	{
		MyRemote.vx = 0;
		MyRemote.vy = 0;
		vz = 0;
		is_online = 0;
	}
#if defined(DT7)
	if (MyRemote.rc_ctrl.Get_Key().R.Get_Now_State())fric_ram_status  = CRAZYRAMMER;
	can.ChasisSendCom1( MyRemote.rc_ctrl.Get_RC().ch[2],MyRemote.rc_ctrl.Get_RC().ch[3], motors[0].Angel_Ecd - YawBias,CarMode, is_online);

#elif defined(FSi6x)
	can.ChasisSendCom1(MyRemote.vx * 2,MyRemote.vy * 2,motors[0].Angel_Ecd - YawBias,CarMode, is_online);
#endif
}
/**
 * @brief 设置遥控器数据
 *
 * 更新遥控器的输入数据，并设置目标角度。
 */
void gimbalc::SetWithRC(void)
{
	MyRemote.update();

	CarMode =  MyRemote.CarMode;

	warning = MyRemote.FricStatus;

	vz = -ChassisYawPid.getOut();

	if (Protect_flag == ONLINE) //在线时，根据遥控器发过来的数据更新目标角度
	{
		YawTarget += MyRemote.yaw_speed * 5 / 1000;
		PihTarget += MyRemote.pih_speed * 5 / 1000;
	}

	if(MyRemote.Control_Mode == KEY_MODE)	CarChoose(TOKEY_MODE);
	else CarChoose(TORC_MODE);

	YawSMC.setJ(0.75);

	switch (CarMode)
	{
	default://随动
	{
		if (Last_CarMode != SUIDONG)
		{
			Yaw_EcdClean();
		}
		if (ChassisYawTarget - motors[0].Angel_Ecd > 180)ChassisYawTarget -= 360; //加减2π
		if (motors[0].Angel_Ecd - ChassisYawTarget > 180)ChassisYawTarget += 360;//优弧劣弧处理
		ChassisYawPid.setTarget(ChassisYawTarget); //YAW_Bias
		break;
	}
	case TUOLUO: //陀螺
	{
		Yaw_EcdClean();
		vz = -50.0f; //恒速小陀螺
		break;
	}
	} //模式切换

	Pih_Limit();
	{
		Pid_In.YawAngle_set = YawTarget;
		Pid_In.PihAngle_set = PihTarget;

		pos_pid[0].setTarget(YawTarget);
		pos_pid[1].setTarget(PihTarget);

		YawSMC.setRef(YawTarget);
	}
	Last_CarMode = CarMode;
#ifdef DR16
	if (MyRemote.rc_ctrl.Get_Key().SHIFT.Get_Now_State() && vz != -ChassisYawPid.getOut()) vz = vz * 2.5; //是否需要分段？ 还是按键定模式
#endif
}

/**
 * @brief 初始化目标
 *
 * 设置目标yaw角度和pitch角度为当前电机的角度。
 */
void gimbalc::TargetInit(void)
{
	YawTarget = motors[0].Angel_All;
	PihTarget = motors[1].Angel_All; //PIH好像有点搜索不到
}

/**
 * @brief 计算电流输出
 *
 * 根据不同的控制算法（MATLAB、SLIDE、NOMEL）计算yaw和pitch的电流输出。
 */
void gimbalc::CurrentCompute()
{
	switch (motors[0].Algorithml)
	{
	case MATLAB:
		can.YawSendCurrent(motors[0].pole * Pid_Out.YawCurrent);
		break;
	case SLIDE:
		can.YawSendCurrent(motors[0].pole * YawSMC.getU());
		break;
	case NOMEL:
		can.YawSendCurrent(motors[0].pole * speed_pid[0].getOut());
		break;
	}

	switch (motors[1].Algorithml)
	{
	case MATLAB:
		can.PitchSendCurrent(motors[1].pole * Pid_Out.PihCurrent);
		break;
	case NOMEL:
		can.PitchSendCurrent(motors[1].pole * speed_pid[1].getOut());
		break;
	}
	can.ShootSendCurrent(shoot.speed_pids[1].getOut(), shoot.speed_pids[2].getOut(), shoot.speed_pids[0].getOut(), 0);
}

//matlab生成控制器 先用一个大循环，后期移植到task
/**
 * @brief 控制循环
 *
 * 定期调用各个控制模块，确保系统按预定行为工作。
 */
void gimbalc::ControlLoop()
{
	FeedDog(); //喂狗 √
	Printf_Test(); //√
	AlgorithmCompute();
	SetWithRC();
	Protect_Mode();
	ChassisComLoop();
	CurrentCompute();
}

/**
 * @brief 打印测试信息
 *
 * 用于调试时输出常用信息。
 */
void gimbalc::Printf_Test(void)
{

	usart_printf("\n");
} //放一些常用的打印

//以下是改变随动方向的函数接口
int16_t Get_ChassisTarget(void)
{
	return omni.Get_Chassistarget();
}

void Set_ChassisTarget(int16_t Target)
{
	omni.Set_Chassistarget(Target);
}

void Set_YawTarget(float Target)
{
	Target += omni.Get_YawTarget();
	omni.Set_YawTarget(Target);
}