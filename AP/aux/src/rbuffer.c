#include "rbuffer.h"

/**
 * @brief Initializes the ring buffer.
 * @param buffer Pointer to the ring buffer structure.
 * @return 0 on success.
 */
int rbuffer_init(RBuffer_t *buffer)
{
    buffer->idxStart = 0;
    buffer->idxEnd = 0;
    return 0;
}

/**
 * @brief Writes data to the ring buffer.
 * @param buffer Pointer to the ring buffer structure.
 * @param data Pointer to the data to write.
 * @param len Number of bytes to write.
 * @return Number of bytes written.
 * Writes no more than the available space in the buffer.
 * if the buffer is full it doesn't write data and returns 0.
 *
 */
int rbuffer_write(RBuffer_t *buffer, const void *data, unsigned len)
{
    unsigned avail = rbuffer_getAvailSpace(buffer);
    unsigned to_write = (len < avail) ? len : avail;
    unsigned written = 0;
    uint8_t *src = (uint8_t *)data;

    while (written < to_write)
    {
        buffer->data[buffer->idxEnd] = src[written];
        buffer->idxEnd = (buffer->idxEnd + 1) % RBUFFER_SIZE;
        written++;
    }
    return written;
}

/**
 * @brief Reads data from the ring buffer.
 * @param buffer Pointer to the ring buffer structure.
 * @param data Pointer to the destination buffer.
 * @param len Number of bytes to read.
 * @return Number of bytes read.
 */
int rbuffer_read(RBuffer_t *buffer, void *data, unsigned len)
{
    unsigned available = (buffer->idxEnd >= buffer->idxStart)
                             ? (buffer->idxEnd - buffer->idxStart)
                             : (RBUFFER_SIZE - buffer->idxStart + buffer->idxEnd);
    unsigned toread = (len < available) ? len : available;
    unsigned nbread = 0;
    uint8_t *dst = (uint8_t *)data;

    while (nbread < toread)
    {
        dst[nbread] = buffer->data[buffer->idxStart];
        buffer->idxStart = (buffer->idxStart + 1) % RBUFFER_SIZE;
        nbread++;
    }
    return nbread;
}

/**
 * @brief Gets the next contiguous block of data available for reading.
 * @param buffer Pointer to the ring buffer structure.
 * @param data Pointer to the address of the block.
 * @param len Pointer to the length of the block.
 * @return 0 on success, -1 if buffer is empty.
 */
int rbuffer_getNextBlock(RBuffer_t *buffer, void **data, unsigned *len)
{
    if (rbuffer_isEmpty(buffer))
    {
        *data = (void *)0;
        *len = 0;
    }
    else if (buffer->idxEnd > buffer->idxStart)
    {
        *data = &buffer->data[buffer->idxStart];
        *len = buffer->idxEnd - buffer->idxStart;
    }
    else
    {
        *data = &buffer->data[buffer->idxStart];
        *len = RBUFFER_SIZE - buffer->idxStart;
    }
    return *len;
}

/**
 * @brief Skips len bytes in the buffer (advances idxStart).
 * @param buffer Pointer to the ring buffer structure.
 * @param len Number of bytes to skip.
 * @return Number of bytes skipped.
 * Skips no more than the available data in the buffer.
 */
int rbuffer_skip(RBuffer_t *buffer, unsigned len)
{
#if 0
    unsigned available = (buffer->idxEnd >= buffer->idxStart)
                             ? (buffer->idxEnd - buffer->idxStart)
                             : (RBUFFER_SIZE - buffer->idxStart + buffer->idxEnd);

    unsigned to_skip = (len < available) ? len : available;
    buffer->idxStart = (buffer->idxStart + to_skip) % RBUFFER_SIZE;
    return to_skip;
#else
    buffer->idxStart = (buffer->idxStart + len) % RBUFFER_SIZE;
#endif
}

/**
 * @brief Gets the available space in the buffer.
 * @param buffer Pointer to the ring buffer structure.
 * @return Number of bytes available for writing.
 */
unsigned rbuffer_getAvailSpace(RBuffer_t *buffer)
{
    if (buffer->idxEnd >= buffer->idxStart)
        return RBUFFER_SIZE - (buffer->idxEnd - buffer->idxStart) - 1;
    else
        return (buffer->idxStart - buffer->idxEnd) - 1;
}

/**
 * @brief Checks if the buffer is full.
 * @param buffer Pointer to the ring buffer structure.
 * @return true if full, false otherwise.
 */
bool rbuffer_isFull(RBuffer_t *buffer)
{
    return ((buffer->idxEnd + 1) % RBUFFER_SIZE) == buffer->idxStart;
}

/**
 * @brief Checks if the buffer is empty.
 * @param buffer Pointer to the ring buffer structure.
 * @return true if empty, false otherwise.
 */
bool rbuffer_isEmpty(RBuffer_t *buffer)
{
    return buffer->idxEnd == buffer->idxStart;
}
