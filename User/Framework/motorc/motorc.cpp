#include "motorc.hpp"

/**
 * @brief 检查电机是否在线
 * @return 在线返回true，不在线返回false
 */
bool MotorC::CheckOnline()
{
    times_++;
    if (times_ > 500)
    {
        online_ = false;
    }
    return online_;
}

/**
 * @brief 获取电机原始数据，在CAN的中断服务函数中使用，将输入端的电机数据存入私有变量中
 */
void MotorC::GetMotorInfo_CANcallback(uint32_t ID, uint8_t* data)
{
    if (ID == master_id_)
    {
        online_  = true;
        times_   = 0;
        ecd_pos_ = (int16_t)(data[0] << 8 | data[1]);
        ecd_vel_ = (int16_t)(data[2] << 8 | data[3]);
        ecd_cur_ = (int16_t)(data[4] << 8 | data[5]);
        ecd_tmp_ = (int16_t)(data[6]);
    }
}

/**
 * @brief 处理电机数据，计算出电机输出端的速度(rpm)和累计角度(°)(位置)
 * @return 无
 */
void MotorC::UpdateMotorInfo()
{
    vel_rpm_ = (float)ecd_vel_ / MOTOR_RATIO; //输出端电机转速

    // 判断编码器位置变化是否跨越了0点，计算电机的圈数 (laps)
    if (ecd_pos_ - ecd_pos_last_ < -6000)
        ++laps_; //正转一圈
    else if (ecd_pos_ - ecd_pos_last_ > 6000)
        --laps_; //反转一圈

    agl_all_ = (float)ecd_pos_ * ENCODER_TO_ANGLE + (float)laps_ * 360.0f;
    agl_all_ = agl_all_ / MOTOR_RATIO; //转化为输出端转过的角度，可以看到角度由pos算出来，而角度就反应的是位置

    ecd_pos_last_ = ecd_pos_;
    agl_last_     = agl_all_;
}

/**
 * @brief 设置目标速度
 * @param vel 目标速度
 */
void MotorC::SetTargetVel(float vel)
{
    target_vel_ = vel;
}

/**
 * @brief 根据目标速度计算目标电流
 * @param vel 目标速度
 * @return 目标电流
 */
float MotorC::GetTargetCur(float vel)
{
    if (!online_) { return 0.0f; }
    target_vel_ = vel;
    pid_vel_.SetTarget(target_vel_, PID_POSITIONAL);
    return pid_vel_.UpdateOutput(vel_rpm_);
}

/**
 * @brief 失能电机
 */
void MotorC::Disable()
{
    pid_vel_.Clear();
}
