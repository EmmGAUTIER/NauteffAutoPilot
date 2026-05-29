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

#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "queue.h"
//#include "semphr.h"
//#include "stream_buffer.h"

#include <stm32l452xx.h>
#include <stm32l4xx_ll_gpio.h>
#include <stm32l4xx_ll_usart.h>

typedef enum {
    BLINK_MSG_TICK = 1,       /*Clock tick send to start flash(es) sequence */
    BLINK_MSG_STOP,           /* stop blinking */
    BLINK_MSG_START,          /* start blinking */
    BLINK_SET_FLASH_DURATION, /* set flash duration */
} Blink_Msg_Type_t;

typedef struct {
    Blink_Msg_Type_t msgType;
    union {
        float duration;
    } data;
} Blink_Msg_t;

void Blink_task_init();
void Blink_task(void *param);
void Blink_stop(void);
void Blink_start(void);
void Blink_set_flash_duration(float duration);
