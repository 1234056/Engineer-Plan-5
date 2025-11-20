#include "main.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "dma.h"
#include "usart.h"
#include "MCUDriver/canio/canio.hpp"
#include "Framework/chassisc/chassisc.hpp"
#include "Framework/start_task/start_task.hpp"
#include "MCUDriver/usartio/usartio.hpp"
#include "MCUDriver/ledio/ledio.hpp"
#include "tim.h"
#include "Framework/remotec/remotec.hpp"

void BSP_Init();

int main()
{
    BSP_Init();
    while (1)
    {
        HAL_Delay(5);
    }
}

//
void BSP_Init()
{
    HAL_RCC_DeInit();
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_TIM5_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();
    CAN_Init();
    USARTIO_DriverInit();
    LEDIO_DriverInit();

    TASK_StartInit();
}
