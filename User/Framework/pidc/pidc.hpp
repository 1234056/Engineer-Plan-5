#ifndef PIDC_HPP
#define PIDC_HPP

// PID类型枚举型
typedef enum
{
    PID_POSITIONAL  = 0, //位置式
    PID_INCREMENTAL = 1  //增量式
} ePidType;

// PID模式枚举型，Mode
typedef enum
{
    PID_NORMAL = 0, //正常模式
    PID_RAMP   = 1  //斜坡模式
} ePidMode;

// 封装实现 PID 控制器的核心数据和函数
class PidC
{
public:
    PidC() = default; //构造默认函数

    PidC(float kp, float ki, float kd) :
        kp_(kp), ki_(ki), kd_(kd)
    {
    }

    //完整参数构造函数
    PidC(float kp, float        ki, float         kd,
         float error_max, float output_max, float output_step_max,
         float ramp_step) :
        kp_(kp), ki_(ki), kd_(kd),
        error_max_(error_max), output_max_(output_max), output_step_max_(output_step_max),
        ramp_step_(ramp_step)
    {
    }

    void  SetTarget(float target, bool mode);
    float UpdateOutput(float input);
    void  Clear();

    float GetOutput() { return output_now_; }
    void updateTarget();

private:

    static inline float LIMIT(float var, float max, float min) //限幅函数
    {
        return ((var) < (min) ? (min) : ((var) > (max) ? (max) : (var)));
    }

    float input_now_  = 0; //当前输入值,rpm
    float target_now_ = 0; //当前目标值,m/s
    float output_now_ = 0; //当前输出值,pid电流,A

    float ramp_step_ = 10; //斜坡步进值（加速度）

    float kp_{}; //比例项系数
    float ki_{}; //积分项系数
    float kd_{}; //微分项系数

    float error_sum_  = 0;      //累计误差值
    float error_now_  = 0;      //当前误差值
    float error_last_ = 0;      //上次误差值
    float error_prev_ = 0;      //上上次误差值
    float error_max_  = 655350; //最大误差值

    float output_last_     = 0;     //上次输出值
    float output_max_      = 10000; //最大输出值
    float output_step_max_ = 10000; //最大步进输出值

    float target_real_ = 0; //斜坡函数需要输出的目标值

    float precision_ = 0; //精确范围

    bool pid_type_ = PID_POSITIONAL; //PID类型
    bool pid_mode_ = PID_RAMP;       //PID模式
};

#endif //PIDC_HPP
