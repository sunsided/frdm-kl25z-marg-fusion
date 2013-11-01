/*
 * io.h
 *
 *  Created on: Nov 1, 2013
 *      Author: Markus
 */

#ifndef IO_H_
#define IO_H_

/**
 * @brief Sends a char without flushing the buffer.
 */
void IO_SendByteUncommited(uint8_t value);

/**
 * @brief Sends a char, flushing the buffer.
 * @param[in] value The char to send
 * 
 * Must only be used after initialization of Uart0 interrupt.
 */
void IO_SendByte(uint8_t value);

/**
 * @brief Sends a 16bit value in native endianness
 * @param[in] value The value to send
 * 
 * Must only be used after initialization of Uart0 interrupt.
 */
void IO_SendInt16(uint16_t value);

/**
 * @brief Sends a 32bit value in native endianness
 * @param[in] value The value to send
 * 
 * Must only be used after initialization of Uart0 interrupt.
 */
void IO_SendInt32(uint32_t value);

/**
 * @brief Sends a string
 * @param[in] string The string to send
 * @param[in] length The string length
 * 
 * Must only be used after initialization of Uart0 interrupt.
 */
void IO_SendString(const char *string, uint8_t length);

/**
 * @brief Sends a buffer
 * @param[in] string The data to send
 * @param[in] length The data length
 * 
 * Must only be used after initialization of Uart0 interrupt.
 */
void IO_SendBuffer(const uint8_t *buffer, uint8_t length);

/**
 * @brief Flushes the IO.
 */
void IO_Flush();

/**
 * @brief Tests if data is available from IO 
 * @return nonzero if data is available.
 * 
 * Must only be used after initialization of Uart0 interrupt.
 */
uint8_t IO_HasData();

/**
 * @brief Reads a byte from IO. 
 * @return The byte or undefined if no data was available.
 * 
 * Must only be used after initialization of Uart0 interrupt.
 * Caution must be taken to not read data when IO_HasData() would
 * return true as this may break internal buffers.
 */
uint8_t IO_ReadByte();

/**
 * @brief Reads an 16bit integer from IO in native endianness. 
 * @return The data or undefined if no data was available.
 * 
 * Must only be used after initialization of Uart0 interrupt.
 * Caution must be taken to not read data when IO_HasData() would
 * return true as this may break internal buffers.
 */
uint16_t IO_ReadInt16();

/**
 * @brief Reads an 32bit integer from IO in native endianness. 
 * @return The data or undefined if no data was available.
 * 
 * Must only be used after initialization of Uart0 interrupt.
 * Caution must be taken to not read data when IO_HasData() would
 * return true as this may break internal buffers.
 */
uint32_t IO_ReadInt32();


#endif /* IO_H_ */
