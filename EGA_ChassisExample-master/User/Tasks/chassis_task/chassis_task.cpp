#include "chassis_task.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "chassisc.hpp"
#include "canio.hpp"
#include "usart.h"
#include "motorc.hpp"
#include "ledc.hpp"

///底盘运动控制循环
void ChassisTask(void* pv)
{
    portTickType current_time;
    uint32_t     run_times = 0;
    while (true)
    {
        current_time = xTaskGetTickCount(); //获取当前tick（时间）
        chassis.ControlLoop();
        // chassis.Print();
        led_c.ledLoop();
        run_times++;
        vTaskDelayUntil(&current_time, pdMS_TO_TICKS(5)); //让任务以 5 毫秒的周期运行
    }
}
