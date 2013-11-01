/*
 * io.c
 *
 *  Created on: Nov 1, 2013
 *      Author: Markus
 */

#include "ARMCM0plus.h"
#include "derivative.h" /* include peripheral declarations */
#include "comm/buffer.h"
#include "comm/uart.h"

#include "nice_names.h"

extern buffer_t* uartReadFifo; /*< the read buffer, initialized by Uart0_InitializeIrq() */
extern buffer_t* uartWriteFifo; /*< the write buffer, initialized by Uart0_InitializeIrq() */

/*
 * TODO: Add variants with defined endianness by reading the AIRCR.ENDIANNESS bit.
 */

/**
 * @brief Sends a char without flushing the buffer.
 */
void IO_SendByteUncommited(uint8_t value)
{
	/* echo to output but don't enable TIE */
	RingBuffer_BlockWhileFull(uartWriteFifo);
	RingBuffer_Write(uartWriteFifo, value);
}

/**
 * @brief Sends a char, flushing the buffer.
 * @param[in] value The char to send
 */
void IO_SendByte(uint8_t value)
{
	/* echo to output */
	RingBuffer_BlockWhileFull(uartWriteFifo);
	RingBuffer_Write(uartWriteFifo, value);
	
	/* enable transmit IRQ */
	Uart0_EnableTransmitIrq();
}

/**
 * @brief Sends a 16bit value in native endianness
 * @param[in] value The value to send
 */
void IO_SendInt16(uint16_t value)
{
	/* echo to output */
	RingBuffer_BlockWhileFull(uartWriteFifo);
	RingBuffer_Write(uartWriteFifo, (value & 0xFF00) >> 8);
	
	RingBuffer_BlockWhileFull(uartWriteFifo);
	RingBuffer_Write(uartWriteFifo, (value & 0x00FF));
	
	/* enable transmit IRQ */
	Uart0_EnableTransmitIrq();
}

/**
 * @brief Sends a 32bit value in native endianness
 * @param[in] value The value to send
 */
void IO_SendInt32(uint32_t value)
{
	/* echo to output */
	RingBuffer_BlockWhileFull(uartWriteFifo);
	RingBuffer_Write(uartWriteFifo, (value & 0xFF000000) >> 24);
	
	RingBuffer_BlockWhileFull(uartWriteFifo);
	RingBuffer_Write(uartWriteFifo, (value & 0x00FF0000) >> 16);
	
	RingBuffer_BlockWhileFull(uartWriteFifo);
	RingBuffer_Write(uartWriteFifo, (value & 0x0000FF00) >> 8);
	
	RingBuffer_BlockWhileFull(uartWriteFifo);
	RingBuffer_Write(uartWriteFifo, (value & 0x000000FF));
	
	/* enable transmit IRQ */
	Uart0_EnableTransmitIrq();
}

/**
 * @brief Sends a string
 * @param[in] string The string to send
 * @param[in] length The string length
 */
void IO_SendString(const char *string, uint8_t length)
{
	/* enqueue the bytes */
	for (uint8_t i=0; i<length; ++i)
	{
		RingBuffer_BlockWhileFull(uartWriteFifo);
		RingBuffer_Write(uartWriteFifo, string[i]);
	}
	
	/* enable transmit IRQ */
	Uart0_EnableTransmitIrq();
}

/**
 * @brief Sends a zero-terminated string
 * @param[in] string The string to send
 */
void IO_SendZString(const char *string)
{
	/* enqueue the bytes */
	register uint8_t i=0;
	for (char c=string[0]; (c=string[i]) != 0; ++i)
	{
		RingBuffer_BlockWhileFull(uartWriteFifo);
		RingBuffer_Write(uartWriteFifo, c);
	}
	
	/* enable transmit IRQ */
	Uart0_EnableTransmitIrq();
}

/**
 * @brief Sends a string
 * @param[in] string The string to send
 * @param[in] length The string length
 */
void IO_SendBuffer(const uint8_t *string, uint8_t length)
{
	/* enqueue the bytes */
	for (uint8_t i=0; i<length; ++i)
	{
		RingBuffer_BlockWhileFull(uartWriteFifo);
		RingBuffer_Write(uartWriteFifo, string[i]); /* TODO: RingBuffer_WriteBlocking() */
	}
	
	/* enable transmit IRQ */
	Uart0_EnableTransmitIrq();
}

/**
 * @brief Flushes the IO.
 */
void IO_Flush()
{
	Uart0_EnableTransmitIrq();
}


/**
 * @brief Tests if data is available from IO 
 * @return nonzero if data is available.
 * 
 * Must only be used after initialization of Uart0 interrupt.
 */
uint8_t IO_HasData()
{
	return !RingBuffer_Empty(uartReadFifo);
}

/**
 * @brief Reads a byte from IO. 
 * @return The byte or undefined if no data was available.
 * 
 * Must only be used after initialization of Uart0 interrupt.
 * Caution must be taken to not read data when IO_HasData() would
 * return true as this may break internal buffers.
 */
uint8_t IO_ReadByte()
{
	RingBuffer_BlockWhileEmpty(uartReadFifo);
	return RingBuffer_Read(uartReadFifo);
}

/**
 * @brief Reads an 16bit integer from IO in native endianness. 
 * @return The data or undefined if no data was available.
 * 
 * Must only be used after initialization of Uart0 interrupt.
 * Caution must be taken to not read data when IO_HasData() would
 * return true as this may break internal buffers.
 */
uint16_t IO_ReadInt16()
{
	RingBuffer_BlockWhileEmpty(uartReadFifo);
	uint8_t high = RingBuffer_Read(uartReadFifo);
	
	RingBuffer_BlockWhileEmpty(uartReadFifo);
	uint8_t low = RingBuffer_Read(uartReadFifo);
	return (((uint16_t)high) << 8) | low;
}

/**
 * @brief Reads an 32bit integer from IO in native endianness. 
 * @return The data or undefined if no data was available.
 * 
 * Must only be used after initialization of Uart0 interrupt.
 * Caution must be taken to not read data when IO_HasData() would
 * return true as this may break internal buffers.
 */
uint32_t IO_ReadInt32()
{
	RingBuffer_BlockWhileEmpty(uartReadFifo);
	uint32_t value = RingBuffer_Read(uartReadFifo);
	
	RingBuffer_BlockWhileEmpty(uartReadFifo);
	value = value << 8 | RingBuffer_Read(uartReadFifo);
	
	RingBuffer_BlockWhileEmpty(uartReadFifo);
	value = value << 8 | RingBuffer_Read(uartReadFifo);
	
	RingBuffer_BlockWhileEmpty(uartReadFifo);
	value = value << 8 | RingBuffer_Read(uartReadFifo);
	return value;
}
