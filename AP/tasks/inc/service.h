
#define SERVICE_QUEUE_LENGTH 20
#define SERVICE_USART_BUFFER_SIZE 128

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t data[SERVICE_USART_BUFFER_SIZE];
    unsigned idxStart;
    unsigned idxEnd;
    SemaphoreHandle_t sem;
} ServiceUartHandle_t;


extern ServiceUartHandle_t svc_uart1;
extern ServiceUartHandle_t svc_uart2;

int svc_UART_Write(ServiceUartHandle_t *svc_uart, const void*data, size_t len, TickType_t delay);
int init_taskService();