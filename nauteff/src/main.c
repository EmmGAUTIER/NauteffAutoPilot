
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

//int int_global;
//int plaf[] = {0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555, 0x55555555};
//char plouf[] = "UUUUUUUUUUUUUUUU ! La grenouille !";
uint32_t compteur = 0;

// Déclaration du handle du timer
TimerHandle_t xTimer;

// Callback du timer
void vTimerCallback(TimerHandle_t xTimer)
{
    compteur++;
    // printf("Timer déclenché!\n");
}


void TaskMain(void* /* parameters*/)
{
    //xTaskCreate(taskDialog, "Dialog", configMINIMAL_STACK_SIZE + 200, NULL, 4, NULL);
    xTaskCreate(taskMEMs, "MEMs", configMINIMAL_STACK_SIZE + 200, NULL, 4, NULL);
    //xTaskCreate(taskMotor, "Motor", configMINIMAL_STACK_SIZE + 200, NULL, 2, NULL);

#if 0
    // Création du timer
    xTimer = xTimerCreate("MonTimer",       // Nom du timer
                          pdMS_TO_TICKS(1000), // Période en ticks (ici 1000 ms)
                          pdTRUE,           // Auto-reload activé
                          (void *)0,        // Identifiant (non utilisé ici)
                          vTimerCallback);  // Fonction de callback

    if (xTimer != NULL)
    {
        // Démarrage du timer avec un délai de 0
        if (xTimerStart(xTimer, 0) != pdPASS)
        {
            for(;;);
        }
    }
#endif

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
    xTaskCreate(TaskMain, "Main", configMINIMAL_STACK_SIZE + 200, NULL, 2, NULL);

    vTaskStartScheduler();

    for (;;)
        ;
}