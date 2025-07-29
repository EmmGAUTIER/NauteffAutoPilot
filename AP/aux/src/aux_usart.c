#include "FreeRTOS.h"
#include "semphr.h"
#include "stream_buffer.h"

#include <stm32l452xx.h>
#include <stm32_assert.h>
//#include "stm32l4xx_ll_gpio.h"

#include "aux_usart.h"

aux_USART_Handle_t aux_usart1_handle = {
    .registers = USART1,
    .semTx = (SemaphoreHandle_t)0,
    .semRx = (SemaphoreHandle_t)0,
    .bufferTx = (StreamBufferHandle_t)0,
    .bufferRx = (StreamBufferHandle_t)0};

aux_USART_Handle_t aux_usart2_handle = {
    .registers = USART2,
    .semTx = (SemaphoreHandle_t)0,
    .semRx = (SemaphoreHandle_t)0,
    .bufferTx = (StreamBufferHandle_t)0,
    .bufferRx = (StreamBufferHandle_t)0};

void aux_USART_Init_all()
{
    aux_USART_Init(aux_usart1);
    aux_USART_Init(aux_usart2);
}

/*
 * @brief Initialize an USART peripheral.
 *
 * This function initializes the USART peripheral and its associated
 * semaphores and buffers. It must be called before using the USART.
 *
 * @param usart Pointer to the USART handle to initialize.
 */

void aux_USART_Init(aux_USART_Handle_t *usart)
{
    usart->semRx = xSemaphoreCreateMutex();
    usart->semTx = xSemaphoreCreateMutex();
    usart->bufferRx = xStreamBufferCreate(aux_USART_BUFFER_SIZE, 1);
    usart->bufferTx = xStreamBufferCreate(aux_USART_BUFFER_SIZE, 1);

    assert_param(usart->semTx != NULL);
    assert_param(usart->semRx != NULL);
    assert_param(usart->bufferTx != NULL);
    assert_param(usart->bufferRx != NULL);

    return;
}

/*
 * @brief Write data to an USART.
 *
 * This function is reentrant. It uses a mutex to ensure
 * exclusive access to the USART's transmit buffer.
 * it waits if buffer is full or another function has called it.
 *
 * @param usart Pointer to the USART handle.
 * @param data Pointer to the data to write.
 * @param len Length of the data to write.
 * @param delay Maximum time to wait for the operation to complete.
 * @return The number of bytes written, or -1 on error.
 */

int aux_USART_write(aux_USART_Handle_t *usart, void *data, size_t len, TickType_t delay)
{
    BaseType_t res;

    res = xSemaphoreTake(usart->semTx, delay);
    if (res == pdFAIL)
    {
        return -1;
    }

    res = xStreamBufferSend(usart->bufferTx, data, len, delay);
    if (res == pdFAIL)
    {
        return -1;
        xSemaphoreGive(usart->semTx);
    }

    xSemaphoreGive(usart->semTx);

    // usart->registers;

    return res;
}

/*
 * @brief Read a character from an USART.
 *
 * This function is not reentrant. Only one task should call it at a time.
 * This function blocks until a character is available or the delay expires.
 *
 * @param usart Pointer to the USART handle.
 * @param delay Maximum time to wait for the operation to complete.
 * @return The character read, or -1 on error.
 */

int aux_USART_getc(aux_USART_Handle_t *usart, TickType_t delay)
{
    char car;
    BaseType_t res;

    res = xStreamBufferReceive(usart->bufferRx, &car, 1, delay);
    if (res == 0)
    {
        return -1; // No character received
    }

    return car; // Return the received character
}

void aux_USART_IRQHandler(aux_USART_Handle_t *usart)
{
    char car;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (LL_USART_IsActiveFlag_TXE(usart->registers))
    {
        xStreamBufferReceiveFromISR(usart->bufferTx, &car, 1, &xHigherPriorityTaskWoken);
        LL_USART_TransmitData8(usart->registers, car);
        if (xStreamBufferIsEmpty(usart->bufferTx))
        {
            LL_USART_DisableIT_TXE(usart->registers);
        }
    }

    taskYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}