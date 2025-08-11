#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include <stm32l452xx.h>
#include "stm32l4xx_ll_gpio.h"

#include "aux_usart.h"
#include "service.h"

#include "motor.h"
#include "mems.h"
#include "apdialog.h"
#include "autopilot.h"

void taskBlink(void *param)
/*
 * \brief Blinks the green LED on the Nucleo board.
 * This task is used for debugging and to check if the system is running.
 */
{
    (void)param;

    for (;;)
    {
        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
        vTaskDelay(pdMS_TO_TICKS(100));

        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
        vTaskDelay(pdMS_TO_TICKS(200));

        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
        vTaskDelay(pdMS_TO_TICKS(100));

        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
        vTaskDelay(pdMS_TO_TICKS(600));
    }
}

void apmain()
{
    int ret;


    init_taskMotor();
    init_taskMEMs();

    init_taskDialogIn();
    init_taskAutoPilot();
    init_taskService();

    xTaskCreate(taskMotor, "Motor", configMINIMAL_STACK_SIZE + 200, (void *)0, 3, (void *)0);
    xTaskCreate(taskMEMs, "MEMs", configMINIMAL_STACK_SIZE + 200, (void *)0, 2, (void *)0);
    xTaskCreate(taskDialogIn, "Dialog", configMINIMAL_STACK_SIZE + 200, (void *)0, 2, (void *)0);
    xTaskCreate(taskAutoPilot, "Auto Pilot", configMINIMAL_STACK_SIZE + 200, (void *)0, 2, (void *)0);
    xTaskCreate(taskService, "SVC", configMINIMAL_STACK_SIZE + 200, (void *)0, 7, (void *)0);

    ret = xTaskCreate(taskBlink, "Blink", configMINIMAL_STACK_SIZE + 200, (void *)0, 2, (void *)0);

    vTaskStartScheduler(); /* Start scheduler, should never return */
    for (;;)
        ;
    (void)ret; // To avoid unused variable warning
}