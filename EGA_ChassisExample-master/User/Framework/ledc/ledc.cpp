/* Private includes ----------------------------------------------------------*/
#include "ledc.hpp"

LedC     led_c;
uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {0xFF0000FF, 0xFF00FF00, 0xFFFF0000, 0xFF0000FF};

/* Exported functions  -------------------------------------------------------*/
extern TIM_HandleTypeDef htim5;

/* Private function prototypes -----------------------------------------------*/
void (*setLedLevel)(LedC led, uint8_t level); //指针函数，可以指向任意一个函数以替代

/* Private user code ---------------------------------------------------------*/
void LED_SetLedOn(LedC& led)
{
    led.state = LED_ON;
    setLedLevel(led, led.on_level);
}

void LED_SetLedOff(LedC& led)
{
    led.state = LED_OFF;
    setLedLevel(led, !led.on_level);
}

void LED_SetLedToggle(LedC& led)
{
    led.state = LED_TOGGLE;
    setLedLevel(led, 3);
}

void LED_RGBInit()
///RGB三色灯初始化
{
    HAL_TIM_Base_Start(&htim5);
    //start pwm channel
    //启用了三个PWM通道，每个通道分别对应R、G、B三色，由于占空比不同，因而不断闪烁
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
}

void LED_SetLedARGB(uint32_t argb)
///设置渐变led灯
{
    static uint8_t  alpha;
    static uint16_t red, green, blue;

    alpha = (argb & 0xFF000000) >> 24;
    red   = ((argb & 0x00FF0000) >> 16) * alpha;
    green = ((argb & 0x0000FF00) >> 8) * alpha;
    blue  = ((argb & 0x000000FF) >> 0) * alpha;

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);
}

/**
  * @brief  设置led的状态
  * @param  status: LED_OFF, LED_ON, LED_TOGGLE
  * @retval None
  */
void LedC::setStatus(LED_State_e status)
{
    this->state = status;
    switch (status)
    {
    case LED_ON:
        LED_SetLedOn(*this);
        break;
    case LED_OFF:
        LED_SetLedOff(*this);
        break;
    case LED_TOGGLE:
        LED_SetLedToggle(*this);
        break;
    default:
        break;
    }
}

void LedC::ledLoop()
///渐变led灯
{
    static uint16_t i,           timer; // `i` 为颜色索引，`timer` 用于计时
    static double   delta_alpha, delta_red, delta_green, delta_blue;
    static double   alpha,       red,       green,       blue; // 当前的颜色分量
    static uint32_t argb;

    //每当计时器到达完整的颜色流周期时，重置颜色索引 `i`
    if (timer % (RGB_FLOW_COLOR_LENGHT * RGB_FLOW_COLOR_CHANGE_TIME) == 0)
        i = 0;

    //如果计时器达到颜色变换时间的间隔，则更新目标颜色和增量
    if (timer % RGB_FLOW_COLOR_CHANGE_TIME == 0)
    {
        //提取当前颜色分量（alpha、red、green、blue）
        alpha = (RGB_flow_color[i] & 0xFF000000) >> 24; //表示透明度，0表示完全透明，255表示完全不透明
        red   = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
        green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
        blue  = ((RGB_flow_color[i] & 0x000000FF) >> 0);

        //计算到下一个颜色的差值（即增量）
        delta_alpha = (double)((RGB_flow_color[i + 1] & 0xFF000000) >> 24) -
            (double)((RGB_flow_color[i] & 0xFF000000) >> 24);
        delta_red = (double)((RGB_flow_color[i + 1] & 0x00FF0000) >> 16) -
            (double)((RGB_flow_color[i] & 0x00FF0000) >> 16);
        delta_green = (double)((RGB_flow_color[i + 1] & 0x0000FF00) >> 8) -
            (double)((RGB_flow_color[i] & 0x0000FF00) >> 8);
        delta_blue = (double)((RGB_flow_color[i + 1] & 0x000000FF) >> 0) -
            (double)((RGB_flow_color[i] & 0x000000FF) >> 0);

        //将增量平分到每个时间步长
        delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;

        i++; //移动到下一个颜色
    }

    //累加增量，实现颜色平滑过渡
    alpha += delta_alpha;
    red += delta_red;
    green += delta_green;
    blue += delta_blue;

    //将计算得到的颜色分量合并成一个 ARGB 颜色
    argb = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 |
        ((uint32_t)(blue)) << 0;
    LED_SetLedARGB(argb);
    timer++;
}
