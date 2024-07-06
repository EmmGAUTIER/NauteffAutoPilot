/*
 * @file Module "bno055"
 * Ce module assure le dialoge avec un capteur BNO055.
 *  -
 *  -
 *  -
 *  -
 *  -
 *  -
 */

#include "usart.h"

extern MessageBufferHandle_t bufferBNO055;

typedef struct
{
    UART_Handle *usart;
    int errno;
} BNO055_Handle;

extern BNO055_Handle bno055_Handle;
#define bno055 (&bno055_Handle)

int BNO055_init(BNO055_Handle *, UART_Handle *uh);

void taskBNO055(void *);

int BNO055_write(BNO055_Handle *device, unsigned reg, char *data, unsigned len);
int BNO055_read(BNO055_Handle *device, unsigned reg, char *data, unsigned len);

INLINE int BNO055_get_errno(BNO055_Handle *device)
{
    return device->errno;
}