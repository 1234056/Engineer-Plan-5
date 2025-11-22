//
// Created by mjw on 2022/9/19.
//
//
#include "main.h"
#include "shoot_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "gimbalc.h"
#include "debugc.h"
#include "iwdgc.h"


void ShootControlTask(void const* argument)
{
	/* USER CODE BEGIN ShootControlTask */
	portTickType CurrentTime;
	/* Infinite loop */
	for (;;)
	{
		CurrentTime = xTaskGetTickCount();
		shoot.ControlLoop();
		vTaskDelayUntil(&CurrentTime, 5 / portTICK_RATE_MS);
	}
	/* USER CODE END ShootControlTask */
}
