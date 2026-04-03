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
    BlinkBlink,
    BlinkOn,
    BlinkOff,
} BlinkMsgType_t;

typedef struct {
    BlinkMsgType_t msgType;
    TickType_t delays[6];
} BLINK_Msg_t;

extern TaskHandle_t BlinkTaskHandle;

int init_taskBlink(void);

void taskBlink(void *pvParameters);

void BLINK_setPeriods(TickType_t* delays);

//extern QueueHandle_t msgQueueBlink;
extern TaskHandle_t BlinkTaskHandle;
