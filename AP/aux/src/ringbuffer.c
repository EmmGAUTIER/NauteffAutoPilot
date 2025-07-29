
#include "stdint.h"

#include "ringbuffer.h"

/*
 * @brief Put data in a ring buffer
 * @param buffer pointer to RingBuffer_t
 * @data pointer to data
 * @len lenght of data to transmit
 * @return len or -1 if not enough space
 */
int RingBufferPut(RingBuffer_t *buffer, const void *data, int len)
{
    const char *src;
    char *dst;
    int idxBuff;

    if (len <= RingBufferSpaceAvailable(buffer))
    {
        src = (const char *)data;
        dst = buffer->data[buffer->idxEnd];
        idxBuff = buffer->idxEnd;
        for (int idxSrc = 0; idxSrc < len; idxSrc++)
        {
            buffer->data[idxBuff++] = *src++;
            if (idxBuff >= RING_BUFFER_SIZE)
            {
                idxBuff = 0;
            }
        }
        return len;
    }
    else
    {
        return -1;
    }
}

int RingBufferGetBlock(RingBuffer_t *buffer, void **pdata, int *size)
{
    ;
}