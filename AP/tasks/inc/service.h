
#include "rbuffer.h"

#define SERVICE_QUEUE_LENGTH 20
#define SERVICE_USART_BUFFER_SIZE 512

/*
 * Service Handle for UART communication.
 * This structure holds the state of the UART and a circular buffer.
 */
typedef struct
{
    UART_HandleTypeDef *huart;   /* Pointer to HAL UART handler */
    RBuffer_t bufferTx;          /* Circular buffer for UART data to transmit */
    volatile size_t dma_tx_busy; /* 0: no DMA in progress, >0 : length of DMA transfer in progress */
    uint8_t charRx;              /* Last character received */
    SemaphoreHandle_t semTx;     /* Protect the streambuffer against concurrent write access */
    SemaphoreHandle_t semRx;     /* Protect the streambuffer against concurrent read access */
    StreamBufferHandle_t receiveBuffer;
} ServiceUartHandle_t;

extern ServiceUartHandle_t svc_uart1;
extern ServiceUartHandle_t svc_uart2;

int svc_UART_Write(ServiceUartHandle_t *svc_uart, const void *data, size_t len, TickType_t delay);
int svc_UART_getc(ServiceUartHandle_t *svc_uart, TickType_t delay);
int init_taskService();
void taskService(void *);
