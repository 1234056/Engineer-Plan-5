#include "chassisc.hpp"
#include "canio.hpp"
#include "user_lib.h"

ChassisC chassis; // 底盘控制类

/**
 * @brief 从云台获取遥控器命令，放在CAN中断里
 * @param ID 云台消息的帧头
 * @param data 云台发送的数据
 */
void ChassisC::GetGimbalCmd(uint32_t ID, uint8_t data[8])
{
    if (ID == 0x107)
    {
        int16_t vx       = (int16_t)(data[0] << 8) | data[1];
        int16_t vy       = (int16_t)(data[2] << 8) | data[3];
        yaw_agl          = (float)(int16_t)((data[4] << 8) | data[5]);
        mode             = (int8_t)data[6];
        gimbal_online_   = (bool)data[7];
        rc_target_vel[X] = (float)vx / 6.6f * 3.0f / 100.0f;
        rc_target_vel[Y] = (float)vy / 6.6f * 3.0f / 100.0f;
        yaw_pid.SetTarget(0.0f, PID_POSITIONAL);
        if (mode == 3)
        {
            rc_target_vel[Z] = yaw_pid.UpdateOutput(yaw_agl);
        }
        else if (mode == 2)
        {
            rc_target_vel[Z] = 100.0f;
        }
        times_ = 0;
    }
}

/**
 * @brief 正解算：由四轮速度解算车体速度，rpm->m/s
 */
void ChassisC::SolveForwardKinematics()
{
    float cos_radz, sin_radz;

    if (chassis_type == MECANUM_WHEEL)
    {
        //单位换算，rpm->rad/now_s
        now_wheel_vel[0] = -_rpmToRad(motors[0].GetVel());
        now_wheel_vel[1] = _rpmToRad(motors[1].GetVel());
        now_wheel_vel[2] = _rpmToRad(motors[2].GetVel());
        now_wheel_vel[3] = -_rpmToRad(motors[3].GetVel());

        //速度正解算：将每个轮子的速度合成为底盘的 x、y 方向速度和旋转角速度
        now_chassis_vel[X] = 0.25f * (-now_wheel_vel[0] + now_wheel_vel[1] - now_wheel_vel[2] + now_wheel_vel[
            3]) * wheel_r_;
        now_chassis_vel[Y] = 0.25f * (now_wheel_vel[0] + now_wheel_vel[1] + now_wheel_vel[2] + now_wheel_vel[3])
            * wheel_r_;
        now_chassis_vel[Z] = _radToAngle(
            0.25f * (now_wheel_vel[0] - now_wheel_vel[1] - now_wheel_vel[2] + now_wheel_vel[3]) * wheel_r_ / (
                length_2_ + width_2_));
    }
    else if (chassis_type == OMNI_WHEEL)
    {
        //单位换算，rpm->rad/now_s
        now_wheel_vel[0] = -_rpmToRad(motors[0].GetVel());
        now_wheel_vel[1] = -_rpmToRad(motors[1].GetVel());
        now_wheel_vel[2] = -_rpmToRad(motors[2].GetVel());
        now_wheel_vel[3] = -_rpmToRad(motors[3].GetVel());

        //正解算
        now_chassis_vel[X] = 0.25f * (-now_wheel_vel[0] - now_wheel_vel[1] + now_wheel_vel[2] + now_wheel_vel[
            3]) * wheel_r_;
        now_chassis_vel[Y] = 0.25f * (now_wheel_vel[0] - now_wheel_vel[1] - now_wheel_vel[2] + now_wheel_vel[3])
            * wheel_r_;
        now_chassis_vel[Z] = _radToAngle(
            0.25f * (now_wheel_vel[0] + now_wheel_vel[1] + now_wheel_vel[2] + now_wheel_vel[3]) * wheel_r_ / (
                length_2_ + width_2_));
    }
    //获取云台角度并转换为弧度
    theta_   = _angleToRad(yaw_agl);
    cos_radz = cosf(theta_);
    sin_radz = sinf(theta_);

    //将底盘坐标系中的速度转换到全局坐标系
    now_abs_vel[X] = cos_radz * now_chassis_vel[X] - sin_radz * now_chassis_vel[Y];
    now_abs_vel[Y] = sin_radz * now_chassis_vel[X] + cos_radz * now_chassis_vel[Y];
    now_abs_vel[Z] = now_chassis_vel[Z];
}

/**
 * @brief 逆解算：由车体速度解算四轮速度，m/s->rpm
 */
void ChassisC::SolveInverseKinematics()
{
    float cos_radz, sin_radz;
    float temp_s[4]; // 全局坐标系下速度，rad/s

    //获取云台角度
    theta_   = _angleToRad(yaw_agl);
    cos_radz = cosf(theta_);
    sin_radz = sinf(theta_);

    //计算全局目标速度
    target_chassis_vel[X] = cos_radz * target_abs_vel[X] + sin_radz * target_abs_vel[Y];
    target_chassis_vel[Y] = -sin_radz * target_abs_vel[X] + cos_radz * target_abs_vel[Y];
    target_chassis_vel[Z] = _angleToRad(target_abs_vel[Z]);

    //逆解算
    if (chassis_type == MECANUM_WHEEL)
    {
        do
        {
            temp_s[0] = (-target_chassis_vel[X] + target_chassis_vel[Y] + target_chassis_vel[Z] * (length_2_ +
                width_2_)) / wheel_r_;
            temp_s[1] = (target_chassis_vel[X] + target_chassis_vel[Y] - target_chassis_vel[Z] * (length_2_ + width_2_))
                / wheel_r_;
            temp_s[2] = (-target_chassis_vel[X] + target_chassis_vel[Y] - target_chassis_vel[Z] * (length_2_ +
                width_2_)) / wheel_r_;
            temp_s[3] = (target_chassis_vel[X] + target_chassis_vel[Y] + target_chassis_vel[Z] * (length_2_ + width_2_))
                / wheel_r_;
            target_chassis_vel[Z] *= 0.8f; //优先降低旋转速度
        }
        while (_isSpdOutRange(temp_s, wheel_spd_max_) && fabsf(target_chassis_vel[Z]) > 6.0f);

        target_wheel_rad[0] = temp_s[0];
        target_wheel_rad[1] = temp_s[1];
        target_wheel_rad[2] = temp_s[2];
        target_wheel_rad[3] = temp_s[3];
        // usart_printf("%f,%f,%f,%f \n",target_rad[0],target_rad[1],target_rad[2],target_rad[3]);
        //单位换算，rad/temp_s->rpm
        target_wheel_vel[0] = -_radToRpm(temp_s[0]);
        target_wheel_vel[1] = _radToRpm(temp_s[1]);
        target_wheel_vel[2] = _radToRpm(temp_s[2]);
        target_wheel_vel[3] = -_radToRpm(temp_s[3]);
    }
    else if (chassis_type == OMNI_WHEEL)
    {
        //逆解算
        do
        {
            temp_s[0] = (-target_chassis_vel[X] + target_chassis_vel[Y] + target_chassis_vel[Z] * (length_2_ +
                width_2_)) / wheel_r_;
            temp_s[1] = (-target_chassis_vel[X] - target_chassis_vel[Y] + target_chassis_vel[Z] * (length_2_ +
                width_2_)) / wheel_r_;
            temp_s[2] = (target_chassis_vel[X] - target_chassis_vel[Y] + target_chassis_vel[Z] * (length_2_ + width_2_))
                / wheel_r_;
            temp_s[3] = (target_chassis_vel[X] + target_chassis_vel[Y] + target_chassis_vel[Z] * (length_2_ + width_2_))
                / wheel_r_;
            // usart_printf("%f,%f,%f,%f \n",temp_s[0],temp_s[1],temp_s[2],temp_s[3]);
            target_chassis_vel[Z] *= 0.8f; //优先降低旋转速度
        }
        while (_isSpdOutRange(temp_s, wheel_spd_max_) && fabsf(target_chassis_vel[Z]) > 9.0f);

        target_wheel_rad[0] = temp_s[0];
        target_wheel_rad[1] = temp_s[1];
        target_wheel_rad[2] = temp_s[2];
        target_wheel_rad[3] = temp_s[3];
        // usart_printf("%f,%f,%f,%f \n",target_rad[0],target_rad[1],target_rad[2],target_rad[3]);
        //单位换算，rad/temp_s->rpm
        target_wheel_vel[0] = -_radToRpm(temp_s[0]);
        target_wheel_vel[1] = -_radToRpm(temp_s[1]);
        target_wheel_vel[2] = -_radToRpm(temp_s[2]);
        target_wheel_vel[3] = -_radToRpm(temp_s[3]);
    }
}

/**
 * @brief 使用遥控器输入更新底盘的目标速度和加速度
 */
void ChassisC::UpdateAcc()
{
    speed_updating = true;
    float delta_vel[4]; //速度差

    //限幅
    for (int i = 0; i < 3; i++)
    {
        rc_target_vel[i] *= v_max_limit_[i];
    }

    //abs_vel是当前车体速度
    delta_vel[X] = rc_target_vel[X] - now_abs_vel[X];
    delta_vel[Y] = rc_target_vel[Y] - now_abs_vel[Y];

    //rc_target_vel[A]是计算出来的合成速度差，而其余X、Y、Z是云台传过来的
    rc_target_vel[A] = sqrtf(delta_vel[X] * delta_vel[X] + delta_vel[Y] * delta_vel[Y]); //合成速度

    //已给定的合成方向加速度和云台Z轴旋转加速度
    target_abs_acc[A] = 1.0f;
    target_abs_acc[Z] = 6.0f;

    //如果目标速度不是零，按 x、y 方向的速度差进行分配加速度
    if (rc_target_vel[A] != 0)
    {
        target_abs_acc[X] = fabsf(target_abs_acc[A] * delta_vel[X] / rc_target_vel[A]) * 0.3f;
        //a[A]实际上是一个限幅，delta_v[X] / rc_v[A]是一个X和Y方向的速度分配比例
        target_abs_acc[Y] = fabsf(target_abs_acc[A] * delta_vel[Y] / rc_target_vel[A]) * 0.3f;
    }

    speed_updating = false;
}

/**
 * @brief 使用斜坡函数更新底盘本轮循环的目标速度
 */
void ChassisC::UpdateVel()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        if (target_abs_vel[i] - rc_target_vel[i] > target_abs_acc[i])
        {
            target_abs_vel[i] -= target_abs_acc[i];
        }
        else if (target_abs_vel[i] - rc_target_vel[i] < -target_abs_acc[i])
        {
            target_abs_vel[i] += target_abs_acc[i];
        }
        else
        {
            target_abs_vel[i] = rc_target_vel[i];
        }
    }
}

/**
 * @brief 发送四轮电机的电流
 * @param c0 左前轮电流
 * @param c1 右前轮电流
 * @param c2 左后轮电流
 * @param c3 右后轮电流
 */
void ChassisC::setMotorsCur(int16_t c0, int16_t c1, int16_t c2, int16_t c3)
{
    uint8_t cmd[8];
    cmd[0] = (uint8_t)(c0 >> 8);
    cmd[1] = (uint8_t)(c0);
    cmd[2] = (uint8_t)(c1 >> 8);
    cmd[3] = (uint8_t)(c1);
    cmd[4] = (uint8_t)(c2 >> 8);
    cmd[5] = (uint8_t)(c2);
    cmd[6] = (uint8_t)(c3 >> 8);
    cmd[7] = (uint8_t)(c3);
    CanSendCmd(CAN_CHASSIS_ALL_ID, cmd);
}


/**
 * @brief 底盘是否在线
 * @return 返回底盘是否在线，包括四轮电机和遥控器
 */
bool ChassisC::CheckOnline()
{
    times_++;
    if (times_ > 100)
    {
        online_ = false;
    }
    online_ = motors[0].CheckOnline()
        && motors[1].CheckOnline()
        && motors[2].CheckOnline()
        && motors[3].CheckOnline()
#ifdef WHOLE_ROBOT
        &&gimbal_online_;
#elifdef CHASSIS_ONLY
        && rc.CheckOnline();
#endif
    return online_;
}

/**
 *@brief 底盘CAN帧接收回调函数
 * @param ID CAN帧ID
 * @param data CAN帧数据
 */
void ChassisC::ChassisCallback(uint32_t ID, uint8_t data[8])
{
    times_  = 0;
    online_ = motors[0].IsMotorOnline()
        && motors[1].IsMotorOnline()
        && motors[2].IsMotorOnline()
        && motors[3].IsMotorOnline();
    motors[0].GetMotorInfo_CANcallback(ID, data);
    motors[1].GetMotorInfo_CANcallback(ID, data);
    motors[2].GetMotorInfo_CANcallback(ID, data);
    motors[3].GetMotorInfo_CANcallback(ID, data);
}

/**
 * @brief 底盘更新函数，包括遥控器、四轮电机的更新
 */
void ChassisC::ChassisUpdate()
{
    for (auto& wheel : motors)
    {
        wheel.UpdateMotorInfo();
    }
}

/**
 * @brief 调试用，可自由配置想打印的东西
 */
void ChassisC::Print()
{
    // usart_printf("%f,%f,%f\r\n",
    //     rc_target_vel[X], rc_target_vel[Y],rc_target_vel[Z]);
    // usart_printf("%d,%d,%d,%d\r\n", target_wheel_cur[0], target_wheel_cur[1], target_wheel_cur[2], target_wheel_cur[3]);
}

/**
 * @brief 底盘控制循环
 */
void ChassisC::ControlLoop()
{
    ChassisUpdate();                        // 更新四轮电机信息
    SolveForwardKinematics();               // 正解算
    rc.UpdateRCTarget(rc_target_vel, &mode); // 更新遥控器输入
    UpdateAcc();                            // 根据遥控器输入，计算目标加速度
    UpdateVel();                            // 使用斜坡函数更新目标速度
    SolveInverseKinematics();               // 逆解算
    for (uint8_t i = 0; i < 4; i++)
    {
        target_wheel_cur[i] = (int16_t)motors[i].GetTargetCur(target_wheel_vel[i]); // 计算目标电流
    }

    if (CheckOnline())
    {
        //usart_printf("%f,%f\r\n", target_wheel_vel[0], now_wheel_vel[0]);
        setMotorsCur(target_wheel_cur[0], target_wheel_cur[1], target_wheel_cur[2], target_wheel_cur[3]);
    }
    else
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            target_wheel_cur[i] = 0;
            motors[i].Disable(); //新增积分项清零
        }
        setMotorsCur(0, 0, 0, 0);
    }
    Print(); // 打印测试信息
}
