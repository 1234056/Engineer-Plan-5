//
// Created by 2b superman on 25-10-25.
//

#include "Gimbal_Task.h"
#include "main.h"
#include "Shoot_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "gimbalc.h"
#include "debugc.h"
#include "iwdgc.h"

void GimbalControlTask(void const* argument)
{
    /* USER CODE BEGIN GimbalControlTask */
    TickType_t CurrentTime;
    const TickType_t PeriodTicks = pdMS_TO_TICKS(5.1);

    /* Infinite loop */
    for (;;)
    {
        CurrentTime = xTaskGetTickCount();
        // omni.ControlLoop();
        vTaskDelayUntil(&CurrentTime, PeriodTicks);
    }
    /* USER CODE END GimbalControlTask */
}