#include "ledio.hpp"
#include "ledc.hpp"
#include "stm32f4xx_hal_gpio.h"

void LEDIO_PortSetLedLevel(LedC led_c, uint8_t level)
///点亮LED，分为三个状态
{
    switch (level)
    {
    case 0:
        HAL_GPIO_WritePin(led_c.led_port, led_c.led_pin, GPIO_PIN_RESET);
        break;
    case 1:
        HAL_GPIO_WritePin(led_c.led_port, led_c.led_pin, GPIO_PIN_SET);
        break;
    default:
        HAL_GPIO_TogglePin(led_c.led_port, led_c.led_pin);
    }
}

/**
  * @brief  用来连接MUC驱动函数和ledc中的控制函数
  * @retval None
  */
//必须先配置，否则调用该函数指针的时候会有问题
void LEDIO_DriverInit()
///LED初始化
{
    LED_RGBInit();
    setLedLevel = LEDIO_PortSetLedLevel; //将LEDIO_PortSetLedLevel替换为setLedLevel
}
