
#define RING_BUFFER_SIZE 256

typedef struct
{
    char data[RING_BUFFER_SIZE];
    unsigned idxStart;
    unsigned idxEnd;
} RingBuffer_t;

INLINE void RingBufferInit(RingBuffer_t *buffer)
{
    buffer->idxStart = 0U;
    buffer->idxEnd = 0U;
}

INLINE size_t RingBufferSpaceAvailable(RingBuffer_t *buffer)
{
    size_t spaceAvailable;

    if (buffer->idxEnd >= buffer->idxStart)
    {
        spaceAvailable = buffer->idxEnd - buffer->idxStart;
    }
    else
    {
        ;
        spaceAvailable = RING_BUFFER_SIZE - (buffer->idxStart - buffer->idxEnd);
    }

    return spaceAvailable;
}
