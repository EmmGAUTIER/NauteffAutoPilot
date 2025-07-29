
#include "FreeRTOS.h"
#include "semphr.h"
#include "stream_buffer.h"

#include "stm32l452xx.h"
#include "stm32l4xx_ll_usart.h"

#include "util.h"

#define aux_USART_BUFFER_SIZE 196

typedef struct
{
    USART_TypeDef *registers;

    SemaphoreHandle_t semTx;
    SemaphoreHandle_t semRx;

    StreamBufferHandle_t bufferTx;
    StreamBufferHandle_t bufferRx;

} aux_USART_Handle_t;

extern aux_USART_Handle_t aux_usart1_handle;
extern aux_USART_Handle_t aux_usart2_handle;

#define aux_usart1 (&aux_usart1_handle)
#define aux_usart2 (&aux_usart2_handle)

void aux_USART_Init_all();
void aux_USART_Init(aux_USART_Handle_t *usart);

int aux_USART_write(aux_USART_Handle_t *usart, void *data, size_t len, TickType_t delay);
int aux_USART_getc(aux_USART_Handle_t *usart, TickType_t delay);

void aux_USART_IRQHandler(aux_USART_Handle_t *usart);