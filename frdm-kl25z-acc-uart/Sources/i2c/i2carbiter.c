/*
 * i2carbiter.c
 *
 *  Created on: Nov 10, 2013
 *      Author: Markus
 */

#include "derivative.h"
#include "bme.h"
#include "i2c/i2carbiter.h"

/**
 * @brief Control structure for the I2C arbiter
 */
typedef struct {
	uint16_t lastSelectedSlave;		/*< The last selected slave address */
	uint8_t lastSelectedSlaveIndex;	/*< The list index of the last selected slave */
	i2carbiter_entry_t* entries;	/*< The arbiter entries */
	const uint8_t entryCount;		/*< The number of arbiter entries */
} i2carbiter_t;

/**
 * @brief The port configuration
 */
static i2carbiter_t configuration;

/**
 * @brief Configures n I2C arbiter entry
 * @param[inout] entry The entry
 * @param[in] port The port to use
 * @param[in] sdaPin The number of the pin used for SDA
 * @param[in] sclPin The number of the pin used for SCL
 */
void I2CArbiter_PrepareEntry(i2carbiter_entry_t *entry, uint8_t slaveAddress, PORT_MemMapPtr port, uint32_t sclPin, uint8_t sclMux, uint32_t sdaPin, uint8_t sdaMux)
{
	entry->port = port;
	*(uint8_t*)&entry->slaveAddress = slaveAddress;
	*(uint32_t*)&entry->sdaPin = sdaPin;
	*(uint8_t*)&entry->sdaMux = sdaMux;
	*(uint32_t*)&entry->sclPin = sclPin;
	*(uint8_t*)&entry->sclMux = sclMux;
}

/**
 * @brief Configures the I2C arbiter
 * @param[inout] config The control structure
 * @param[in] entries The entries
 * @param[in] entryCount the number of entries
 */
void I2CArbiter_Configure(i2carbiter_entry_t *entries, uint8_t entryCount)
{
	configuration.lastSelectedSlave = 0;
	configuration.entries = entries;
	*(uint32_t*)&configuration.entryCount = entryCount;
	
	/* assume the first slave will be used first */ 
	I2CArbiter_Select(entries[0].slaveAddress);
	
	/* TODO: This would be a good place to reset the I2C bus on ALL pins */
}

/**
 * @brief Selects an I2C slave and prepares the ports.
 * @param[in] slaveAddress The slave address
 * @return Zero if successful, nonzero otherwise
 */
uint8_t I2CArbiter_Select(uint8_t slaveAddress)
{
	register uint32_t lastSelectedSlave = configuration.lastSelectedSlave;
	
	/* early exit */
	if (lastSelectedSlave == slaveAddress) 
	{
		return 0;
	}
	
	/* find the current slave */
	register int count = configuration.entryCount;
	for (int i=0; i<count; ++i)
	{
		i2carbiter_entry_t* token = &configuration.entries[i]; 
		if (token->slaveAddress == slaveAddress)
		{
			/* disable last slave */
			if (lastSelectedSlave != 0)
			{
				/* disable last selected slave */
				i2carbiter_entry_t* entry = &configuration.entries[configuration.lastSelectedSlaveIndex];
				entry->port->PCR[entry->sdaPin] &= ~PORT_PCR_MUX_MASK;
				entry->port->PCR[entry->sclPin] &= ~PORT_PCR_MUX_MASK;
			}

			/* TODO: candidate for BME */
			
			/* enable new slave */
			token->port->PCR[token->sdaPin] &= ~PORT_PCR_MUX_MASK;
			token->port->PCR[token->sdaPin] |= PORT_PCR_MUX(token->sdaMux);
			token->port->PCR[token->sclPin] &= ~PORT_PCR_MUX_MASK;
			token->port->PCR[token->sclPin] |= PORT_PCR_MUX(token->sclMux);
			
			/* TODO: use BME here */
			
			/* set up lookup */
			configuration.lastSelectedSlave = slaveAddress;
			configuration.lastSelectedSlaveIndex = i;
			
			return 0;
		}
	}
	
	return 1;
}
