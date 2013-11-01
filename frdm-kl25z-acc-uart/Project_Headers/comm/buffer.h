/*
 * buffer.h
 *
 *  Created on: Nov 1, 2013
 *      Author: Markus
 */

#ifndef BUFFER_H_
#define BUFFER_H_

/**
 * @brief Ring buffer type
 */
typedef struct {
	const uint32_t size;	/*< The data array size; Power of two */
	const uint32_t mask;	/*< The address mask; one less than size */
	uint32_t writeIndex;	/*< The write index */
	uint32_t readIndex;		/*< The read index */
	uint8_t */*const*/ data;	/*< The data array; Size is defined in size */
} buffer_t;

/**
 * @brief Resets the ring buffer
 * @param[in] buffer The ring buffer instance
 * @param[in] data The data buffer to be used; Length must be at least a size of two.
 * @param[in] size The data buffer size; Must be a size of two
 */
static uint8_t RingBuffer_Init(buffer_t *buffer, uint8_t (*data)[], const uint32_t size)
{
	// only allow buffers sizes that are a power of two
	if ((size & (size-1)) != 0) 
	{
		return 1;
	}
	
	*(uint8_t**)(&buffer->data) = (uint8_t*)*data;
	*(uint32_t*)(&buffer->size) = size;
	*(uint32_t*)(&buffer->mask) = size-1;
	buffer->writeIndex = buffer->readIndex = 0;
	
	return 0;
}

/**
 * @brief Resets the ring buffer
 * @param[in] buffer The ring buffer instance
 */
__STATIC_INLINE void RingBuffer_Reset(buffer_t *buffer)
{
	buffer->writeIndex = buffer->readIndex = 0;
	__DMB();
}

/**
 * @brief Writes an item to the ring buffer
 * @param[in] buffer The ring buffer instance
 * @param[in] data The data to write
 */
__STATIC_INLINE void RingBuffer_Write(buffer_t *buffer, const uint8_t data)
{
	buffer->data[buffer->mask & (buffer->writeIndex++)] = data;
	__DMB();
}

/**
 * @brief Reads an item from the ring buffer
 * @param[in] buffer The ring buffer instance
 * @return The data read
 */
__STATIC_INLINE const uint8_t RingBuffer_Read(buffer_t *buffer)
{
	const uint8_t data = buffer->data[buffer->mask & (buffer->readIndex++)];
	__DMB();
	return data;
}

/**
 * @brief Returns if the ring buffer is empty
 * @param[in] buffer The ring buffer instance
 * @return nonzero if empty, zero otherwise
 */
__STATIC_INLINE const uint8_t RingBuffer_Empty(buffer_t *buffer)
{
	__DMB();
	/* no masking required here because indices are free running and only masked on access */
	return (buffer->readIndex == buffer->writeIndex);
}

/**
 * @brief Gets the count of unread items in the buffer
 * @param[in] buffer The ring buffer instance
 * @return The item count
 */
__STATIC_INLINE const uint32_t RingBuffer_Count(buffer_t *buffer)
{
	__DMB();
	/* no masking required here because indices are free running and only masked on access */
	return /*buffer->mask &*/ (uint32_t)(buffer->writeIndex - buffer->readIndex);
}

/**
 * @brief Returns if the ring buffer is full
 * @param[in] buffer The ring buffer instance
 * @return nonzero if full, zero otherwise
 */
__STATIC_INLINE const uint8_t RingBuffer_Full(buffer_t *buffer)
{
	return RingBuffer_Count(buffer) >= buffer->size;
}

#endif /* BUFFER_H_ */
