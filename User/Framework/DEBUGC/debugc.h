#ifndef ROBO_TEST_CORE_INC_UART8_H_
#define ROBO_TEST_CORE_INC_UART8_H_

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdint.h"

#define DEBUG_RVSIZE 255

class DebugC {
private:
    float vel_kp_;
    float vel_ki_;
    float vel_kd_;
    int32_t vel_maxOutput_;
    int32_t vel_maxIntegral_;
    int32_t vel_rampTargetValue_;
    int32_t vel_rampTargetTime_;
    int32_t vel_rampTargetStep_;

    float pos_kp_;
    float pos_ki_;
    float pos_kd_;
    int32_t pos_maxOutput_;
    int32_t pos_maxIntegral_;
    int32_t pos_maxOutStep_;
    int32_t pos_targetAngle_;

public:
    // 速度环参数访问方法
    float getVelKp() const { return vel_kp_; }
    void setVelKp(float value) { vel_kp_ = value; }
    
    float getVelKi() const { return vel_ki_; }
    void setVelKi(float value) { vel_ki_ = value; }
    
    float getVelKd() const { return vel_kd_; }
    void setVelKd(float value) { vel_kd_ = value; }
    
    int32_t getVelMaxOutput() const { return vel_maxOutput_; }
    void setVelMaxOutput(int32_t value) { vel_maxOutput_ = value; }
    
    int32_t getVelMaxIntegral() const { return vel_maxIntegral_; }
    void setVelMaxIntegral(int32_t value) { vel_maxIntegral_ = value; }
    
    int32_t getVelRampTargetValue() const { return vel_rampTargetValue_; }
    void setVelRampTargetValue(int32_t value) { vel_rampTargetValue_ = value; }
    
    int32_t getVelRampTargetTime() const { return vel_rampTargetTime_; }
    void setVelRampTargetTime(int32_t value) { vel_rampTargetTime_ = value; }
    
    int32_t getVelRampTargetStep() const { return vel_rampTargetStep_; }
    void setVelRampTargetStep(int32_t value) { vel_rampTargetStep_ = value; }

    // 位置环参数访问方法
    float getPosKp() const { return pos_kp_; }
    void setPosKp(float value) { pos_kp_ = value; }
    
    float getPosKi() const { return pos_ki_; }
    void setPosKi(float value) { pos_ki_ = value; }
    
    float getPosKd() const { return pos_kd_; }
    void setPosKd(float value) { pos_kd_ = value; }
    
    int32_t getPosMaxOutput() const { return pos_maxOutput_; }
    void setPosMaxOutput(int32_t value) { pos_maxOutput_ = value; }
    
    int32_t getPosMaxIntegral() const { return pos_maxIntegral_; }
    void setPosMaxIntegral(int32_t value) { pos_maxIntegral_ = value; }
    
    int32_t getPosMaxOutStep() const { return pos_maxOutStep_; }
    void setPosMaxOutStep(int32_t value) { pos_maxOutStep_ = value; }
    
    int32_t getPosTargetAngle() const { return pos_targetAngle_; }
    void setPosTargetAngle(int32_t value) { pos_targetAngle_ = value; }
};

#define STARTPID  0x70//小写p
#define STARTLQR 0x6C//小写l
#define START 0x31//字符1
#define STOP  0x30//字符0
#define MAOHAO  0x3A//冒号
#define CONTROL 0x63  //小写c

#define VEL_LOOP 0x73 //小写s，speed
#define VEL_KP 0x70//小写p
#define VEL_KI 0x69//小写i
#define VEL_KD 0x64//小写d
#define VEL_MAXOUT 0x6F//小写o
#define VEL_MAXINTEGRAL 0x61//小写a
#define VEL_TARVALUE 0x76//小写v
#define VEL_TARTIME 0x74//小写t
#define VEL_TARSTEP 0x73//小写s，step

#define POS_LOOP 0x70 //小写p，position
#define POS_KP 0x70//小写p
#define POS_KI 0x69//小写i
#define POS_KD 0x64//小写d
#define POS_MAXOUT 0x6F//小写o
#define POS_MAXINTEGRAL 0x61//小写a
#define POS_MAXSTEP 0x73//小写s，step
#define POS_TARVALUE 0x76 //小写v

extern DebugC debugParam;

extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

void DEBUGC_UartInit();
void DEBUGC_UartIdleCallback(UART_HandleTypeDef *huart);

#endif