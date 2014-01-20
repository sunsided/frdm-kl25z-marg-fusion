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
#include "comm/io.h"

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
void IO_SendInt16(int16_t value)
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
 * @brief Sends a 16bit value as string
 * @param[in] value The value to send
 * 
 * Must only be used after initialization of Uart0 interrupt.
 */
void IO_SendSInt16AsString(int16_t value)
{
	/* if value is negative, print sign and invert */
	if (value < 0) 
	{
		IO_SendByteUncommited('-');
		value = -value;
	}
	
	/* send as unsigned */
	IO_SendUInt16AsString((uint16_t)value);
}

/**
 * @brief Sends a 16bit value as string
 * @param[in] value The value to send
 * 
 * Must only be used after initialization of Uart0 interrupt.
 */
void IO_SendUInt16AsString(uint16_t value)
{
	char buf[5]; /* 5 digits */
	char *p = &buf[0];
	int8_t digits = 0;
	
	/* divide value by 10 until value is 0 */
	do 
	{
		int16_t remainder = value % 10;
		*p++ = (remainder < 10) ? (remainder + '0') : (remainder + 'a' - 10);
		++digits;
	} while (value /= 10);
	
	/* dump buffer */
	do
	{
		IO_SendByte(*--p);
	} while (--digits > 0);
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
void IO_SendZString(const char *const string)
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
void IO_SendBuffer(const uint8_t *const string, uint8_t length)
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

/**
 * @brief Sends a signed number in 2.12 format (signed 1.12)
 * @param[in] input The value to send
 * 
 * Must only be used after initialisation of Uart0 interrupt.
 */
void IO_Send2p14AsString(register int16_t input)
{
	/* Basic idea
	 * ----------
	 * To convert a number in m.n fixed-point format to 
	 * floating point, it is cast to floating point and 
	 * then scaled by 2^-n.
	 * 
	 * Problem
	 * -------
	 * 4096*2^-12 = 1.0	 		<--> 4096 >> 12 = 1
	 * 4095*2^-12 = 0.999755	<--> 4095 >> 12 = 0 ! missing digits
	 * 
	 * Solution
	 * --------
	 * If we first scale the number by powers of 10 (shifting
	 * in the decimal places from the right) and then shift right
	 * by 12 bits ... 
	 * 
	 * 4095*10000 >> 12  = 09997
	 * 
	 * we get to the right digit. If we now subtract the result
	 * of the previous scale, we get the digit alone:
	 * 
	 * (4095*10000) >> 12 - (4095*1000 >> 12) = 7
	 * 
	 * Unfortunately, for later decimal places these calculations
	 * exceed 16 bit number range easily, so we will cast to
	 * unsigned 32 bit integer first.
	 * 
	 * Note
	 * ----
	 * Since even the fourth decimal place can not be represented 
	 * correctly anymore, only six decimal places will be rendered. 
	 */
	
#define Q2p12_M  2
#define Q2p12_N 12	

	register uint8_t digit;
	
	register uint32_t input_value = input;
	register uint32_t current_value = input;
	register uint32_t last_value = 0;
	
	if (input < 0) 
	{
		IO_SendByteUncommited('-');
		input_value = -input_value;
	}
	else 
	{
		IO_SendByteUncommited(' ');
	}
	
	/* proceed with the absolute value in 1.12 format 
	 * and determine real part. 
	 */
	current_value = (input_value) >> Q2p12_N;
	digit = current_value;
	last_value = current_value;
	
	IO_SendByte(digit + '0');
	IO_SendByte('.');
	
	/* now determine the remaining digits */
	current_value = (input_value*10) >> Q2p12_N;
	digit = current_value - last_value*10;
	last_value = current_value;
	IO_SendByte(digit + '0');
	
	current_value = (input_value*100) >> Q2p12_N;
	digit = current_value - last_value*10;
	last_value = current_value;
	IO_SendByte(digit + '0');
	
	current_value = (input_value*1000) >> Q2p12_N;
	digit = current_value - last_value*10;
	last_value = current_value;
	IO_SendByte(digit + '0');
	
	current_value = (input_value*10000) >> Q2p12_N;
	digit = current_value - last_value*10;
	last_value = current_value;
	IO_SendByte(digit + '0');
	
	current_value = (input_value*100000) >> Q2p12_N;
	digit = current_value - last_value*10;
	last_value = current_value;
	IO_SendByte(digit + '0');
	
	current_value = (input_value*1000000) >> Q2p12_N;
	digit = current_value - last_value*10;
	last_value = current_value;
	IO_SendByte(digit + '0');

#undef Q2p12_M
#undef Q2p12_N
}
