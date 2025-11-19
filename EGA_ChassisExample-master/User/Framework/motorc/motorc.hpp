#ifndef MOTORC_HPP
#define MOTORC_HPP

#include "cstdint"
#include "pidc.hpp"

#define CAN_1 1
#define CAN_2 2

#define MOTOR_RATIO         (3591 / 187.0f)     //电机减速比19左右，减速比越大速度越小但是扭矩更大
#define ENCODER_TO_ANGLE    (360 / 8192.0f)     //编码器脉冲数 -> 转动角度(度)         360°/8192脉冲数

#define M3508_KP            150.0f
#define M3508_KI            1.0f
#define M3508_KD            0.0f
#define M3508_ERR_MAX       20000.0f
#define M3508_OUT_MAX       16384.0f
#define M3508_STEP_MAX      5000.0f
#define M3508_TARGET_STEP   1.0f

// 封装电机核心数据
class MotorC
{
public:
    MotorC() = default; //构造默认函数

    MotorC(uint32_t id, uint32_t master_id, int8_t can_line, float motor_ratio) : //CAN发送ID，接收ID，CAN通道，电机减速比
        id_(id), master_id_(master_id), can_line_(can_line), motor_ratio_(motor_ratio)
    {
    }

    void  GetMotorInfo_CANcallback(uint32_t ID, uint8_t* data);
    void  UpdateMotorInfo();
    float GetTargetCur(float vel); // 返回目标电流，单位为 A

    bool CheckOnline(); // 检查电机是否在线
    void Disable();     // 停止电机，清除PID
    void Print()        // 打印测试信息
    {
        // 说词!
    }

    [[nodiscard]] float GetAngle() const { return agl_all_; }
    [[nodiscard]] float GetVel() const { return vel_rpm_; }
    [[nodiscard]] float GetTorue() const { return (float)ecd_cur_ * motor_ratio_; } // 返回电流，单位为 A
    [[nodiscard]] bool IsMotorOnline() const { return online_; }
    [[nodiscard]] int16_t GetId() const { return id_; }

    void SetTargetVel(float vel); // 设置目标转速，单位为 rpm

private:

    PidC pid_vel_ = PidC(M3508_KP, M3508_KI, M3508_KD, M3508_ERR_MAX, M3508_OUT_MAX, M3508_STEP_MAX,
                         M3508_TARGET_STEP); //速度控制器

    bool online_ = true; //电机是否在线

    float target_vel_ = 0.0f;                //目标转速，单位为 rpm

    int32_t times_ = 0; //计数器，用于记录特定状态或事件的次数

    float   agl_all_  = 0; //电机累计角度
    float   agl_last_ = 0; //上一次记录的角度
    float   vel_rpm_  = 0; //电机转速，单位为 rpm
    int32_t laps_     = 0; //电机累计转动的圈数

    //添加了ecd的为输入端，它与输出端的比值为减速比
    int16_t ecd_pos_      = 0;
    int16_t ecd_pos_last_ = 0;
    int16_t ecd_vel_      = 0;
    int16_t ecd_cur_      = 0;
    int16_t ecd_tmp_      = 0;

    float    motor_ratio_ = MOTOR_RATIO; //减速比
    int8_t   can_line_    = CAN_1;
    uint32_t master_id_   = 0x200; // 接收CAN帧的ID
    uint32_t id_          = 0x200; // 发送CAN帧的ID
};

#endif // MOTORC_HPP
