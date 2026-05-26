#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include <stm32l452xx.h>
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_tim.h" // pour mise au point
#include "stm32l4xx_hal.h"    // pour mise au point

#include "aux_usart.h"
#include "service.h"

#include "motor.h"
#include "ahrs.h"
#include "mems.h"
#include "apdialog.h"
#include "autopilot.h"
#include "blink.h"

TaskHandle_t tasksHandles [] = {
        [0] = NULL, /* taskMotor */
        [1] = NULL, /* taskMEMs */
        [2] = NULL, /* taskDialogIn */
        [3] = NULL, /* taskAutoPilot */
        [4] = NULL, /* taskService */
        [5] = NULL  /* taskBlink */
};
int tasksNumber = sizeof(tasksHandles) / sizeof(tasksHandles[0]);

extern int _sram2;
extern int _eram2;
/*
 * @brief blink a LED for debugging purpose only
 *
 * Blinks the green LED on the Nucleo board.
 * This task is used for debugging and to check if the system is running.
 *
 */

void taskBlink(void *param)
{
    (void)param;

    for(;;) /* infinite loop */
    {
        /*set and reset a LED with delays */
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

void panic(int panicType)
{
    MOTOR_stopPanic();
    /* Alarm_give(); */
    /* TODO fire a reset witth special condition */
}

void apmain()
{
    int ret = 0;

    /* init_tasksXXX fcts create queues semaphores, timers,...
     * before starting scheduler
     */
    init_taskMotor();
    init_taskMEMs();
    init_taskDialogIn();
    init_taskAutoPilot();
    init_taskService();

    /* Create tasks */
    ret &= xTaskCreate(taskMotor,     "Motor",      configMINIMAL_STACK_SIZE + 500, (void *)0, 3, tasksHandles + 0);
    ret &= xTaskCreate(taskMEMs,      "MEMs",       configMINIMAL_STACK_SIZE + 500, (void *)0, 4, tasksHandles + 1);
    ret &= xTaskCreate(taskDialogIn,  "Dialog",     configMINIMAL_STACK_SIZE + 300, (void *)0, 2, tasksHandles + 2);
    ret &= xTaskCreate(taskAutoPilot, "Auto Pilot", configMINIMAL_STACK_SIZE + 500, (void *)0, 2, tasksHandles + 3);
    ret &= xTaskCreate(taskService,   "SVC",        configMINIMAL_STACK_SIZE + 200, (void *)0, 5, tasksHandles + 4);
    ret &= xTaskCreate(taskBlink,     "Blink",      configMINIMAL_STACK_SIZE + 100, (void *)0, 2, tasksHandles + 5);

    /* Start scheduler, should never return */
    vTaskStartScheduler();

    for(;;)
        ; /* Shall never happen */

    (void)ret; // To avoid unused variable warning
}