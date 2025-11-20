#include "pidc.hpp"

/**
 * 位置式PID核心，得到输出值
 * @param input   PID目标值
 * @return output PID输出值
 */
float PidC::UpdateOutput(float input)
{
    updateTarget();

    //计算当前误差
    input_now_ = input;
    //    error_prev = error_last;    //位置式PID不需要
    error_last_ = error_now_;
    error_now_  = target_now_ - input_now_;

    //如果误差在设定的精度范围内，则视为达到目标
    if (error_now_ < precision_ && error_now_ > -precision_)
        error_now_ = 0;

    //累积误差值，用于积分项计算，并限制其范围（防止积分过大）
    error_sum_ += error_now_;
    error_sum_ = LIMIT(error_sum_, error_max_, -error_max_);

    //位置式PID核心公式
    //output_k=Kp⋅ek+Ki∑ei+Kd(ek−ek−1)
    output_now_ = kp_ * error_now_ + ki_ * error_sum_ + kd_ * (error_now_ - error_last_);

    //输出限幅
    output_now_ = LIMIT(output_now_, output_max_, -output_max_);

    //微分限幅
    output_now_ = LIMIT(output_now_, output_last_ + output_step_max_, output_last_ - output_step_max_);

    //将当前输出值保存为上次输出，以备下一次调用使用
    output_last_ = output_now_;

    //返回当前计算出的输出值
    return output_now_;
}

/**
 * 设置目标值
 * @param target 目标值
 * @param mode   正常模式PID_NORMAL/斜坡模式PID_RAMP
 */
void PidC::SetTarget(float target, bool mode)
{
    pid_mode_    = mode;
    target_real_ = target;
}

/**
 * @brief   清空PID参数，防止电机上线时直接疯车
 */
    void PidC::Clear()
{
    target_now_  = 0;
    input_now_   = 0;
    output_now_  = 0;
    output_last_ = 0;
    error_now_   = 0;
    error_sum_   = 0;
    error_last_  = 0;
    error_prev_  = 0;
}

/**
 * @brief   使用斜坡函数，步进更新目标值，每调用一次就会产生一次新的目标值
 */
void PidC::updateTarget()
{
    if (pid_mode_ == PID_RAMP)
    {
        if (target_now_ - target_real_ < -ramp_step_)
        {
            target_now_ += ramp_step_;
        }
        else if (target_now_ - target_real_ > ramp_step_)
        {
            target_now_ -= ramp_step_;
        }
        //如果目标值和斜坡函数需要的一致，则回到普通模式
        else
        {
            target_now_ = target_real_;
        }
    }
    else
    {
        target_now_ = target_real_;
    }
}
