
#include <math.h>
#include <string.h>
#include "FreeRTOS.h"
#include <message_buffer.h>
#include "task.h"
#include "timers.h"
#include "util.h"
#include "gpio.h"
#include "dialog.h"
#include "mems.h"
#include "nauteff.h"
#include "motor.h"

int int_global;
int plaf[] = {0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555};
char plouf[] = "UUUUUUUUUUUUUUUU ! La grenouille !";


void TaskMain(void *args)
{
    xTaskCreate(taskDialog, "Main", configMINIMAL_STACK_SIZE + 200, NULL, 4, NULL);
    xTaskCreate(taskMEMs, "MEMs", configMINIMAL_STACK_SIZE + 200, NULL, 4, NULL);

    for (;;)
    {
        GPIO_pin_set(GPIOA, 5);
        GPIO_pin_set(GPIOA, 2);
        vTaskDelay(pdMS_TO_TICKS(50));
        GPIO_pin_reset(GPIOA, 5);
        GPIO_pin_reset(GPIOA, 2);
        vTaskDelay(pdMS_TO_TICKS(50));
        GPIO_pin_set(GPIOA, 5);
        vTaskDelay(pdMS_TO_TICKS(50));
        GPIO_pin_reset(GPIOA, 5);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main()
{
    xTaskCreate(TaskMain, "Main", configMINIMAL_STACK_SIZE + 200, NULL, 4, NULL);

    vTaskStartScheduler();

    for (;;)
        ;
}