/*
 * Comp_Hal.c
 *
 * Created: 16.7.2014 16:03:09
 *  Author: JR
 * Hardware Abstraction Layer - Hal
 
 */ 

#include <asf.h>
#include "Mag-Hal.h"
#include "FreeRTOS_V_8_Header.h"


void Mag_read(unsigned char Adress, unsigned char *Data, unsigned char Length)
{
	twi_package_t packet_read = {
		.addr         = Adress,      // TWI slave memory address data
		.addr_length  = sizeof (uint8_t),    // TWI slave memory address data size
		.chip         = LIS_ADRESS,      // TWI slave bus address
		.buffer       = Data,        // transfer data destination buffer
		.length       = Length                    // transfer data size (bytes)
	};
	
	taskENTER_CRITICAL();
	twi_master_read(TWI0, &packet_read);
	taskEXIT_CRITICAL();
}

/********************************************************/

void Mag_send(unsigned char Adress, unsigned char *Data, unsigned char Length)
{
	twi_package_t packet_write = {
		.addr         = Adress,      // TWI slave memory address data
		.addr_length  = sizeof (uint8_t),    // TWI slave memory address data size
		.chip         =LIS_ADRESS,      // TWI slave bus address
		.buffer       = Data, // transfer data source buffer
		.length       = Length  // transfer data size (bytes)
	};
	
	taskENTER_CRITICAL();
	twi_master_write(TWI0, &packet_write);
	taskEXIT_CRITICAL();
	
}