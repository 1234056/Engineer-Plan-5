#ifndef CHASSISC_HPP
#define CHASSISC_HPP

#include "pidc.hpp"
#include "motorc.hpp"
#include "cmath"
#include "canio.hpp"
#include "usart.h"
#include "remotec.hpp"

// TODO: 根据需要调整
#define CHASSIS_ONLY
// #define WHOLE_ROBOT

#define CHASSIS_KP          150.0f
#define CHASSIS_KI          1.0f
#define CHASSIS_KD          0.0f
#define CHASSIS_ERR_MAX     20000.0f
#define CHASSIS_OUT_MAX     16384.0f
#define CHASSIS_STEP_MAX    5000.0f
#define CHASSIS_TARGET_STEP 10.0f


// 底盘运动方向枚举型
typedef enum
{
    X = 0, //X轴正方向为右方
    Y = 1, //Y轴正方向为前方
    Z = 2, //Z轴正方向为逆时针
    A = 3  //X、Y构成的矢量合成方向
} eChassisDir;

// 四轮编号枚举型
typedef enum
{
    RT = 0, //0号轮为右前轮
    LT = 1, //1号轮为左前轮
    LB = 2, //2号轮为左后轮
    RB = 3  //3号轮为右后轮
} eWheelNum;

// 底盘解算类型枚举型
typedef enum
{
    MECANUM_WHEEL = 0, //麦克纳姆轮
    OMNI_WHEEL    = 1  //全向轮
} eSolveDir;

// 封装了底盘运动控制的核心数据和方法
class ChassisC
{
public:
    ChassisC() = default;

    bool CheckOnline();
    void Print();
    void ControlLoop();
    void ChassisCallback(uint32_t ID, uint8_t data[8]);
    void GetGimbalCmd(uint32_t ID, uint8_t data[8]);



    cRemote rc = cRemote(&huart3, RC_FRAME_LENGTH, DMA_CPLT_IT); //遥控器对象

    PidC yaw_pid = PidC(5.0f, 0.0f, 0.1f);

private:
    MotorC motors[4] = {
        MotorC(CAN_CHASSIS_ALL_ID, CAN_3508_M1_ID, 2, MOTOR_RATIO),
        MotorC(CAN_CHASSIS_ALL_ID, CAN_3508_M2_ID, 2, MOTOR_RATIO),
        MotorC(CAN_CHASSIS_ALL_ID, CAN_3508_M3_ID, 2, MOTOR_RATIO),
        MotorC(CAN_CHASSIS_ALL_ID, CAN_3508_M4_ID, 2, MOTOR_RATIO),
    };
    // 全局坐标系
    float rc_target_vel[4]  = {0, 0, 0, 0}; //遥控器传入的全局坐标系下车体目标速度
    float now_abs_vel[4]    = {0, 0, 0, 0}; //全局坐标系下车体全局速度
    float target_abs_acc[4] = {0, 0, 0, 0}; //全局坐标系下车体目标加速度
    float target_abs_vel[4] = {0, 0, 0, 0}; //全局坐标系下车体目标速度
    // 底盘坐标系
    float now_chassis_vel[4]    = {0, 0, 0, 0}; //底盘坐标系下车体当前速度
    float target_chassis_vel[4] = {0, 0, 0, 0}; //底盘坐标系下车体目标速度
    // 四轮
    float   target_wheel_rad[4] = {0, 0, 0, 0}; //四轮目标速度,rad/s
    float   target_wheel_vel[4] = {0, 0, 0, 0}; //四轮目标速度,rpm
    float   now_wheel_vel[4]    = {0, 0, 0, 0}; //四轮当前速度,rad/s
    int16_t target_wheel_cur[4] = {0, 0, 0, 0}; //四轮需要设置的电流,A
    // 云台
    float yaw_agl = 0; //云台yaw轴角度
    float pih_agl = 0; //云台pitch轴角度

    // TODO: 根据实际做更改！！！
    eSolveDir chassis_type = OMNI_WHEEL;

    // TODO: 根据实际做更改！！！
    float wheel_r_  = 0.08; //轮子半径，单位m
    float width_2_  = 0.15; //车体半宽，单位m
    float length_2_ = 0.15; //车体半长，单位m

    float theta_         = 0;     //云台偏移全局坐标的角度
    float wheel_spd_max_ = 80.0f; //最大角速度

    float v_max_limit_[3] = {1.0f, 1.0f, 1.0f}; //速度最大限制
    friend struct PowerObj;

    bool    online_        = false; //电机是否在线
    bool    gimbal_online_ = false; //云台是否在线
    uint8_t gimbal_rc_s    = 0;     //云台遥控器模式数据
    int32_t times_         = 0;
    bool    speed_updating = false;
    int16_t mode           = 0; //模式选择：跟随 or 小陀螺

    void ChassisUpdate();
    void UpdateAcc();
    void UpdateVel();
    void SolveForwardKinematics();
    void SolveInverseKinematics();
    void setMotorsCur(int16_t c0, int16_t c1, int16_t c2, int16_t c3);
};

extern ChassisC chassis;

#endif //CHASSISC_HPP
