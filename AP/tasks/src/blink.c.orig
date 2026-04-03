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

#include "blink.h"

TaskHandle_t BlinkTaskHandle = NULL;

static unsigned delays =
        { 500U, 500U, 0U, 0U, 0U, 0U };
static QueueHandle_t msgQueueBlink = NULL;

void init_taskBlink()
{
    /* MEMs task queue creation */
    msgQueueBlink = xQueueCreate(1, sizeof(BLINK_Msg_t));
    TaskAbortDelay();
}

/*
 * @brief blink a LED for debugging purpose only
 *
 * Blinks the green LED on the Nucleo board.
 * This task is used for debugging and to check if the system is running.
 *
 */

void taskBlink(void *param)
{
    (void) param;
    BaseType_t ret;
    int tour = 0;
    BLINK_Msg_t message;

    for (;;) /* infinite loop */
    {

        if (delays[tour] != 0)
        {
            vTaskDelay(pdMS_TO_TICKS(delays[tour]));
            ret = xQueueReceive(msgQueueMEMs, &message, 0);
            if (ret == pdPASS)
            {
                tour = 0;
                //message.
            }
            else
            {
                (tour % 2) ?
                             LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9) :
                             LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
                tour++;
                tour %= 6;
            }

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
