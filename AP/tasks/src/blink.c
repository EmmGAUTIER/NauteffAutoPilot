/*
 MIT License

 Copyright (c) 2025 Emmanuel Gautier / Nauteff

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

/****************************************************************************\
*    Blink task : blink a LED for debugging and testing                      *
******************************************************************************
* This module is used for testing and debugging and demonstrate the use      *
* of task, message queue and timer.                                          *
* It may be studied to understand other tasks.                               *
*                                                                            *
\****************************************************************************/

/* defines to tune the algorithms.
 * They should be defined near the top of file to be easily accessible.
 * Here we define the period of the timer.
 */

 #define BLINK_PERIOD_MS 2000

#include "blink.h"

static QueueHandle_t Blink_Queue_Msg = NULL;
static TimerHandle_t Blink_Timer = NULL;


void Blink_stop(void)
{

    /* message struct may be static because it is never changed */
    static Blink_Msg_t msg = {.msgType=BLINK_MSG_STOP};

    xQueueSend(Blink_Queue_Msg, &msg, pdMS_TO_TICKS(0));

    return;
}

void Blink_start(void)
{

    /* message struct may be static because it is never changed */
    static Blink_Msg_t msg = {.msgType=BLINK_MSG_STOP};

    xQueueSend(Blink_Queue_Msg, &msg, pdMS_TO_TICKS(0));

    return;
}

void Blink_set_flash_duration(float duration)
{
    Blink_Msg_t msg;
    BaseType_t ret;

    msg.msgType = BLINK_SET_FLASH_DURATION;
    msg.data.duration = duration;

    xQueueSend(Blink_Queue_Msg, &msg, pdMS_TO_TICKS(0));

    return;
}

/*
 * @brief Sends a tick message to the MEMs task
 *
 * It is The callback of the timer Blink_Timer.
 * it is called every BLINK_PERIOD_MS milliseconds .
 *
 * This function sends a tick message to the Blink task.
 *
 * @ param xTimer The timer handle (not used)
 * @ return None
 */
void Blink_Timer_Callback(TimerHandle_t xTimer)
{
    (void) xTimer; /* statement to avoid unused parameter warning */

    /* The message is contained in a struct.
     * it is declared static to avoid using stack.
     * Only one blink task, so only one timer and callback execution at a time,
     * so no risk of overwriting the message before it is sent.
     */
    static Blink_Msg_t command =
            {
                    .msgType = BLINK_MSG_TICK
            };

    xQueueSend(Blink_Queue_Msg, (const void* )&command, (TickType_t )0);

    return;
}

/*
 * @brief Creates necessary stuff for the Blink tast.
 *
 * @param none
 * @return none
 *
 * Tasks have to use queues and timers. These have to be created
 * before starting scheduler so when tasks begin they can use them. 
 * The queue is used to send ticks and messages to the task.
 *
 */
void Blink_task_init()
{
    /* MEMs task queue creation */
    Blink_Queue_Msg = xQueueCreate(1, sizeof(Blink_Msg_t));

    /* Create a timer that sends messages periodically to the task */
    Blink_Timer = xTimerCreate("BLINK",
                  pdMS_TO_TICKS(BLINK_PERIOD_MS),
                  pdTRUE, /* Auto reload (repeat indefinitely) */
                  (void*) 0, /* Timer ID, not used */
                  Blink_Timer_Callback);

}

/*
 * @brief blink a LED for debugging purpose only
 *
 * Blinks the green LED on the Nucleo board.
 * This task is used for debugging and to check if the system is running.
 *
 */

void Blink_task(void *param)
{
    (void) param;
    BaseType_t ret;
    Blink_Msg_t message;

    for (;;) /* infinite loop */
    {

        ret = xQueueReceive(Blink_Queue_Msg, &message, pdMS_TO_TICKS(0));
        if (ret == pdPASS)
        {
                ;
        }

        /* We use delays to make very simple code */
        /*set and reset a LED with delays */
        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9);
        vTaskDelay(pdMS_TO_TICKS(100));

        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
        vTaskDelay(pdMS_TO_TICKS(200));

        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9);
        vTaskDelay(pdMS_TO_TICKS(100));

        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
        vTaskDelay(pdMS_TO_TICKS(600));
    }

    /* Mustn't be reached */

}
