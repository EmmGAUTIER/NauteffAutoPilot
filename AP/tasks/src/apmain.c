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
        [0] = NULL, /* taskMotor     */
        [1] = NULL, /* taskMEMs      */
        [2] = NULL, /* taskDialogIn  */
        [3] = NULL, /* taskAutoPilot */
        [4] = NULL, /* taskService   */
        [5] = NULL  /* taskBlink     */
};
int tasksNumber = sizeof(tasksHandles) / sizeof(tasksHandles[0]);

//extern int _sram2;
//extern int _eram2;

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
    Motor_task_init();
    Mems_task_init();
    Dialog_task_init();
    AutoPilot_task_init();
    Service_task_init();
    Blink_task_init();

    /* Create tasks */
    ret &= xTaskCreate(Motor_task,     "Motor",      configMINIMAL_STACK_SIZE + 500, (void *)0, 3, tasksHandles + 0);
    ret &= xTaskCreate(Mems_task,      "MEMs",       configMINIMAL_STACK_SIZE + 500, (void *)0, 4, tasksHandles + 1);
    ret &= xTaskCreate(Dialog_task,    "Dialog",     configMINIMAL_STACK_SIZE + 300, (void *)0, 2, tasksHandles + 2);
    ret &= xTaskCreate(AutoPilot_task, "Auto Pilot", configMINIMAL_STACK_SIZE + 500, (void *)0, 2, tasksHandles + 3);
    ret &= xTaskCreate(Service_task,   "SVC",        configMINIMAL_STACK_SIZE + 200, (void *)0, 5, tasksHandles + 4);
    ret &= xTaskCreate(Blink_task,     "Blink",      configMINIMAL_STACK_SIZE + 100, (void *)0, 2, tasksHandles + 5);

    /* Start scheduler, should never return */
    vTaskStartScheduler();

    for(;;)
        ; /* Shall never happen */

    (void)ret; // To avoid unused variable warning
}
