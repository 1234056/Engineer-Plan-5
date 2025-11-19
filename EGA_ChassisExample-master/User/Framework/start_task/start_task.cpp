//
// Created by Cola on 2022/9/14.
//

#include "start_task.hpp"
#include "cmsis_os.h"
#include "chassis_task.hpp"
#include "ledc.hpp"

void TASK_CreateTask(void* pv);

void TASK_TestTask(void* pv);

void BSP_LoopTask(void* pv);

TaskHandle_t task_create_handle;
TaskHandle_t test_task_handle;
TaskHandle_t chassis_task_handle;
TaskHandle_t bsp_task_handle;

void TASK_StartInit()
{
    BaseType_t xReturn;
    taskENTER_CRITICAL();
    xReturn = xTaskCreate((TaskFunction_t)TASK_CreateTask,
                          "CreateTask",
                          128,
                          (void*)NULL,
                          2,
                          &task_create_handle);
    taskEXIT_CRITICAL();
    if (pdFALSE != xReturn)
    {
        vTaskStartScheduler();
    }
}

void TASK_CreateTask(void* pv)
{
    BaseType_t xReturn;
    taskENTER_CRITICAL();
    xTaskCreate((TaskFunction_t)TASK_TestTask,
                "TestTask",
                512,
                (void*)NULL,
                4,
                &test_task_handle);

    xTaskCreate((TaskFunction_t)ChassisTask,
                "ChassisTask",
                1024,
                (void*)NULL,
                5,
                &chassis_task_handle);

    xReturn = xTaskCreate((TaskFunction_t)BSP_LoopTask,
                          "BSPTask",
                          256,
                          (void*)NULL,
                          6,
                          &bsp_task_handle);

    taskEXIT_CRITICAL();
    if (pdFALSE != xReturn)
        vTaskDelete(task_create_handle);
}

void BSP_LoopTask(void* pv)
{
    uint32_t control_times = 0;
    while (true)
    {
        ++control_times;
        vTaskDelay(5);
    }
}

int16_t cmd_control = 0;

void TASK_TestTask(void* pv)
{
    while (true)
    {
        vTaskDelay(500);
    }
}
