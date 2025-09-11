
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
#include "stream_buffer.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

#include <stm32l452xx.h>
#include <stm32l4xx_ll_gpio.h>
#include <stm32l4xx_ll_usart.h>
#include "main.h"
#include "stm32l4xx_hal.h"

#include "rbuffer.h"
#include "service.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

QueueHandle_t svcQueueRequests = (QueueHandle_t)0;

typedef enum
{
    ServiceUartWrite = 0,
    ServiceUartEndWriteDMA,
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
    void *deviceHandle;
    union
    {
        ServiceTransfer_t transfer;
    } requestdescription;
} ServiceRequest_t;

ServiceUartHandle_t svc_uart1 =
    {
        .huart = &huart1,
        .dma_tx_busy = 0U,
        .charRx = 0U,
        .receiveBuffer = (StreamBufferHandle_t)0,
};

ServiceUartHandle_t svc_uart2 =
    {
        .huart = &huart2,
        .dma_tx_busy = 0U,
        .charRx = 0U,
        .receiveBuffer = (StreamBufferHandle_t)0,
};

int svc_init_UART(ServiceUartHandle_t *svc_uart)
{
    svc_uart->receiveBuffer = xStreamBufferCreate((size_t)SERVICE_USART_BUFFER_SIZE , 1);
    rbuffer_init(&svc_uart->bufferTx);
    svc_uart->dma_tx_busy = 0U;
    svc_uart->semRx = xSemaphoreCreateMutex();
    svc_uart->semTx = xSemaphoreCreateMutex();
    xSemaphoreGive(svc_uart->semRx);
    xSemaphoreGive(svc_uart->semTx);
    return 1;
}

int svc_internal_UART_Write(ServiceRequest_t *request);
int svc_internal_UART_EndWriteDMA(ServiceRequest_t *request);

#if 0
int scv_init_UARTS()
{
    svc_init_UART(&svc_uart1);
    svc_init_UART(&svc_uart2);

    return 0;
}
#endif

int init_taskService()
{
    svcQueueRequests = xQueueCreate(SERVICE_QUEUE_LENGTH,
                                    sizeof(ServiceRequest_t));
    if (svcQueueRequests == pdFAIL)
    {
        return -1;
    }

    svc_init_UART(&svc_uart1);
    svc_init_UART(&svc_uart2);

    return 1;
}

int((*fcts[])(ServiceRequest_t *)) = {
    svc_internal_UART_Write,
    svc_internal_UART_EndWriteDMA};

/*
 * @brief Task that serves io and peripherals requests
 * @param pvParameters: Not used
 * @retval None
 *
 * This task is created by the main task and runs indefinitely.
 * It processes requests from the service queue.
 * This task is designed to handle requests in a non-blocking manner,
 * allowing it to efficiently manage multiple requests without
 * blocking the main execution flow. It uses a queue to receive
 * requests and processes them based on their type.
 * The user sends requests whith the service functions.
 */

void taskService(void *)
{
    ServiceRequest_t request;
    BaseType_t ret;
    int res;

    /* Enable reception by interrupts  of UARTS */

    HAL_UART_Receive_IT(svc_uart1.huart, &(svc_uart1.charRx), 1);
    HAL_UART_Receive_IT(svc_uart2.huart, &(svc_uart2.charRx), 1);

    for (;;)
    {
        ret = xQueueReceive(svcQueueRequests, &request, portMAX_DELAY);
        if (ret != pdFAIL)
        {
            /*
             * The request to serve is ian index to the functions array.
             * The use of a switch statement is less efficient.
             */
            if (request.type >= 0 && request.type < ServiceEnd)
            {
                res = (*fcts[request.type])(&request);
                (void)res;
            }
        }
    }
}

int svc_UART_Write(ServiceUartHandle_t *svc_uart, const void *data, size_t len, TickType_t delay)
{
    // BaseType_t ret;
    UART_HandleTypeDef *huart; /* HAL UART Handle */
    size_t availSpace;
    size_t sendSize;
    static unsigned dejaoccupe = 0;
    static unsigned relance = 0;
    void *dataDMA = 0;
    size_t lenDMA = 0;

    vTaskSuspendAll();

    huart = svc_uart->huart;

    availSpace = rbuffer_getAvailSpace(&(svc_uart->bufferTx));
    if (len > availSpace)
    {
        sendSize = 0U;
    }
    else
    {
        rbuffer_write(&svc_uart->bufferTx, data, len);
        if (svc_uart->dma_tx_busy == 0)
        {
            char deb[4];
            rbuffer_getNextBlock(&svc_uart->bufferTx, &dataDMA, &lenDMA);
            deb[0] = *((char*)dataDMA);
            deb[1] = *((char*)dataDMA+1);
            deb[2] = *((char*)dataDMA+2);
            deb[3] = *((char*)dataDMA+3);
            HAL_UART_Transmit_DMA(huart, dataDMA, lenDMA);
            svc_uart->dma_tx_busy = lenDMA; /* Set DMA transfer busy flag */
            relance++;
            (void)relance;
        }
        else
        {
            dejaoccupe++;
            (void)dejaoccupe;
        }
        sendSize = len;
    }

    xTaskResumeAll();

    return sendSize;
}

int svc_UART_getc(ServiceUartHandle_t *svc_uart, TickType_t delay)
{
    size_t ret;
    int car = 0;

    ret = xStreamBufferReceive(svc_uart->receiveBuffer, &car, 1, delay);

    return (ret == 0) ? 0 : car;
}

int svc_internal_UART_EndWriteDMA(ServiceRequest_t *request)
{
    ServiceUartHandle_t *svc_uart; /* Service UART Handle with HAL ptr and buffer */
    UART_HandleTypeDef *huart;     /* HAL UART Handle */

    vTaskSuspendAll();

    svc_uart = (ServiceUartHandle_t *)request->deviceHandle;
    huart = svc_uart->huart;
    rbuffer_skip(&svc_uart->bufferTx, svc_uart->dma_tx_busy);
    svc_uart->dma_tx_busy = 0; /* Reset DMA transfer busy flag */
    if (!rbuffer_isEmpty(&svc_uart->bufferTx))
    {
        void *data;
        size_t len;
        rbuffer_getNextBlock(&svc_uart->bufferTx, &data, &len);
        HAL_UART_Transmit_DMA(huart, data, len);
        svc_uart->dma_tx_busy = len; /* Set DMA transfer busy flag */
    }

    xTaskResumeAll();

    return 1;
}

/*
 * @brief Copies data and initiates DMA transmission over UART
 * @param request: Pointer to the ServiceRequest_t structure containing the request informations
 */
int svc_internal_UART_Write(ServiceRequest_t *request)
{
    static unsigned trop = 0U;
    ServiceUartHandle_t *svc_uart; /* Service UART Handle with HAL ptr and buffer */
    UART_HandleTypeDef *huart;     /* HAL UART Handle */
    // size_t remaining;              /* remaining space in circular buffer */
    // size_t reqlen;                 /* Requested data length to write */
    // size_t transferlen;            /* Length of data that can be transfered */
    size_t buffput; /* Length of data put in buffer */

    svc_uart = (ServiceUartHandle_t *)request->deviceHandle;
    huart = svc_uart->huart;

    if (rbuffer_getAvailSpace(&svc_uart->bufferTx) < request->requestdescription.transfer.len)
    {
        trop++;
        return 0;
    }

    buffput = rbuffer_write(&svc_uart->bufferTx,
                            request->requestdescription.transfer.data,
                            request->requestdescription.transfer.len);

    if ((buffput > 0) && (svc_uart->dma_tx_busy == 0))
    /* Data added in buffer and no DMA transfer in progress */
    /* initiate a DMA transfer*/
    {

        HAL_UART_StateTypeDef state = HAL_UART_GetState(huart);
        if (state & 0x1)
        {
            int bidon = 0;
            (void)bidon;
        }

        void *data;
        size_t len;
        rbuffer_getNextBlock(&svc_uart->bufferTx, &data, &len);
        HAL_UART_Transmit_DMA(huart, data, len);
        svc_uart->dma_tx_busy = len; /* Set DMA transfer busy flag */
    }

    return buffput;
}
/*
 * @brief Callback function called when UART transmission is complete
 * @param huart: Pointer to the UART_HandleTypeDef of the UART.
 * @return none
 * This function is called by HAL when DMA transmission is complete.
 * It sets the dma_tx_busy flag to 0 for the corresponding UART handle.
 *
 *
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

    static ServiceRequest_t request = {
        .type = ServiceUartEndWriteDMA};
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Cherche le handle de service correspondant
    ServiceUartHandle_t *svc_uart = NULL;
    if (huart == svc_uart1.huart)
    {
        svc_uart = &svc_uart1;
    }
    else if (huart == svc_uart2.huart)
    {
        svc_uart = &svc_uart2;
    }

    /* End of a DMA transfer, sends a notification to service task */
    if (svc_uart != NULL)
    {
        BaseType_t res = HAL_UART_GetState(svc_uart->huart);

        request.deviceHandle = svc_uart;
        xQueueSendFromISR(svcQueueRequests, &request, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*
 * @brief Callback called by HAL interrupt function
 * This callback has to call the xxxFromISR() functions of FreeRTOS
 *
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    ServiceUartHandle_t *svc_uart = (ServiceUartHandle_t *)0;
    if (huart == svc_uart1.huart)
    {
        svc_uart = &svc_uart1;
    }
    else if (huart == svc_uart2.huart)
    {
        svc_uart = &svc_uart2;
    }

    if (huart != (UART_HandleTypeDef *)0)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xStreamBufferSendFromISR(svc_uart->receiveBuffer,
                                 &(svc_uart->charRx),
                                 1,
                                 &xHigherPriorityTaskWoken);
        /* Relance de la réception de caractères */
        HAL_UART_Receive_IT(svc_uart->huart, &svc_uart->charRx, 1); // Réarmement
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
