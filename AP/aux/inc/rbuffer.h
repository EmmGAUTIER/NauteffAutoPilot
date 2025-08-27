#include <stdint.h>
#include <stdbool.h>

#ifndef RBUFFER_H
#define RBUFFER_H

#define RBUFFER_SIZE 512

typedef struct
{
    uint8_t data[RBUFFER_SIZE]; /* Data */
    unsigned idxStart;          /* Start index */
    unsigned idxEnd;            /* End index */
} RBuffer_t;

int rbuffer_init(RBuffer_t *buffer);
int rbuffer_write(RBuffer_t *buffer, const void *data, unsigned len);
int rbuffer_read(RBuffer_t *buffer, void *data, unsigned len);
int rbuffer_getNextBlock(RBuffer_t *buffer, void **data, unsigned *len);
int rbuffer_skip(RBuffer_t *buffer, unsigned len);
unsigned rbuffer_getAvailSpace(RBuffer_t *buffer);
bool rbuffer_isFull(RBuffer_t *buffer);
bool rbuffer_isEmpty(RBuffer_t *buffer);


#endif /* RBUFFER_H */
