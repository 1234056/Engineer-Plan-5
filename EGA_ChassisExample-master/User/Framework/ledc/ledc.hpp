#ifndef INC_2023INFANTRYCHASSIS_LEDC_H
#define INC_2023INFANTRYCHASSIS_LEDC_H

/* Includes ------------------------------------------------------------------*/
#include "cstdint"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

/* Exported defines ----------------------------------------------------------*/
#define LED_Port_IO         GPIO_TypeDef*
#define LED_Port_Pin        uint16_t
#define RGB_FLOW_COLOR_CHANGE_TIME  500     //渐变中每一阶段的时间间隔
#define RGB_FLOW_COLOR_LENGHT   3           //长度为3，所以渐变三个颜色后就会重新开始


/* Exported types ------------------------------------------------------------*/
typedef enum
    ///LED灯状态，灭亮闪烁
{
    LED_OFF = 0,
    LED_ON,
    LED_TOGGLE
} LED_State_e;

/* Exported functions prototypes ---------------------------------------------*/
class LedC
    ///定义一个led实例
{
public:
    LED_State_e state = LED_OFF;
    uint8_t     on_level{}; //点亮电平

    LED_Port_IO  led_port{}; //端口
    LED_Port_Pin led_pin{};  //引脚

    LedC() = default;;

    LedC(LED_Port_IO led_Io, LED_Port_Pin led_Pin, uint8_t on_Level = 0):
        led_port(led_Io), led_pin(led_Pin), on_level(on_Level)
    {
    };

    void setStatus(LED_State_e status);
    void ledLoop();

private:
};

void LED_RGBInit();

/* Exported functions  -------------------------------------------------------*/
// 指向外部一个操作底层的函数
extern void (*setLedLevel)(LedC led, uint8_t level);
extern LedC   led_c;

#endif //INC_2023INFANTRYCHASSIS_LEDC_H
