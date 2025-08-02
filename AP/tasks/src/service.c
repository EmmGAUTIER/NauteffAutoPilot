
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
#include "message_buffer.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

#include <stm32l452xx.h>
#include <stm32l4xx_ll_gpio.h>
#include <stm32l4xx_ll_usart.h>
#include "main.h"
#include "stm32l4xx_hal.h"

#include "service.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

QueueHandle_t svcQueueRequests = (QueueHandle_t)0;

typedef enum
{
    ServiceNone,
    ServiceUartWrite,
    ServiceUartRead,
    ServiceEnd
} ServiceType;

typedef enum
{
    ServiceDirectionIn,
    ServiceDirectionOut
} ServiceDirection;

typedef struct
{
    ServiceDirection dir;
    void *data;
    size_t len;
} ServiceTransfer_t;

typedef struct
{
    ServiceType type;
    void* deviceHandle;
    union
    {
        ServiceTransfer_t transfer;
    } requestdescription;
} ServiceRequest_t;

#if 0
typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t data[SERVICE_USART_BUFFER_SIZE];
    unsigned idxStart;
    unsigned idxEnd;
    SemaphoreHandle_t sem;
} ServiceUartHandle_t;
#endif

ServiceUartHandle_t svc_uart1 =
    {
        .huart = &huart1,
        .idxStart = 0U,
        .idxEnd = 0U,
        .sem = 0U};

ServiceUartHandle_t svc_uart2 =
    {
        .huart = &huart1,
        .idxStart = 0U,
        .idxEnd = 0U,
        .sem = 0U};

int svc_init_UART(ServiceUartHandle_t *svc_uart)
{
    svc_uart->sem = xSemaphoreCreateBinary();
    if (svc_uart->sem == NULL)
    {
        return -1;
    }
    xSemaphoreGive(svc_uart->sem);

    return 1;
}

int svc_internal_UART_Write(ServiceRequest_t *request);

int scv_init_UARTS()
{
    svc_init_UART(&svc_uart1);
    svc_init_UART(&svc_uart2);

    return 0;
}

int((*fcts[])(ServiceRequest_t *)) = {
    svc_internal_UART_Write};

int init_taskService()
{
    svcQueueRequests = xQueueCreate(SERVICE_QUEUE_LENGTH,
                                    sizeof(ServiceRequest_t));
    if (svcQueueRequests != pdFAIL)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}

void taskService(void *)
{
    ServiceRequest_t request;
    BaseType_t ret;
    int res;
    for (;;)
    {
        ret = xQueueReceive(svcQueueRequests, &request, portMAX_DELAY);
        if (ret != pdFAIL)
        {
            if (request.type > 0 && request.type < ServiceEnd)
            {
                res = (*fcts[request.type])(&request);
                (void)res;
            }
        }
    }
}

int svc_internal_UART_Write(ServiceRequest_t *request)
{

    return 0;
}

int svc_UART_Write(ServiceUartHandle_t *svc_uart, const void *data, size_t len, TickType_t delay)
{
    BaseType_t ret;

    ServiceRequest_t request;

    /* There is no check that data are transfered */

    request.type = ServiceUartWrite;
    request.deviceHandle = svc_uart;
    request.requestdescription.transfer.data = (void*)data;
    request.requestdescription.transfer.len = len;
    request.requestdescription.transfer.dir = ServiceDirectionOut;

    ret = xQueueSend(svcQueueRequests, &request, delay);

    return (ret == pdPASS) ? len : -1;
}
