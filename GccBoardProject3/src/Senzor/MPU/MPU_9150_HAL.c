/*
 * MPU_9150_HAL.c
 *
 * Created: 02.12.2015 21:52:00
 *  Author: uzivatel
 */ 


#define MPU_ACC_ADR

#include <asf.h>
#include "MPU_9150_HAL.h"
#include "MPU_9150.h"
#include "FreeRTOS_V_8_Header.h"
#include "Periph_init.h"

extern struct spi_device device_mpu;
#define ADDR_9150_MAG	0X0C

uint32_t MPU_9150_send(unsigned char Adress, unsigned char Length, unsigned char *Data)
{	
	taskENTER_CRITICAL();
	uint8_t status=0;
	spi_select_device(SPI, &device_mpu);
	//ioport_set_pin_level(SX1276_CS_PIN, false);
	spi_put(SPI, Adress & 0x7F);
	status=spi_write_packet(SPI,Data,Length);
	
	spi_deselect_device(SPI, &device_mpu);
	taskEXIT_CRITICAL();
	
	return status;
	
}
/**************************************/

uint32_t MPU_9150_read(unsigned char Adress, short Length, unsigned char *Data)
{	
	uint8_t status=0;
	taskENTER_CRITICAL();
	
	spi_select_device(SPI, &device_mpu);
	spi_put(SPI, Adress | 0x80);
	status=spi_read_packet(SPI,Data,Length);

	//ioport_set_pin_level(SX1276_CS_PIN, true);
	spi_deselect_device(SPI, &device_mpu);
	taskEXIT_CRITICAL();
	return status;
	
}


uint32_t MPU_9150_send_mag(unsigned char Adress, unsigned char Length, unsigned char *Data)
{
	taskENTER_CRITICAL();

	twi_package_t packet_read = {
		.chip         =  ADDR_9150_MAG,                        // TWI slave bus address
		.addr         = Adress,								// TWI slave memory address data
		.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
		.buffer       = Data,                        // transfer data destination buffer
		.length       = Length  // transfer data size (uint8_ts)

	};
	// Perform a multi-uint8_t read access then check the result.
	//
	twi_master_read(TWI0, &packet_read);
	
	taskEXIT_CRITICAL();
}

uint32_t MPU_9150_read_mag(unsigned char Adress, unsigned char Length, unsigned char *Data)
{
	taskENTER_CRITICAL();
	
	//uint8_t temp=0;
	//temp=*Data;
	
	twi_package_t packet_write = {
		.chip         =  ADDR_9150_MAG,                        // TWI slave bus address
		.addr         = Adress,								// TWI slave memory address data
		.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
		.buffer       = Data,                        // transfer data destination buffer
		.length       = Length  // transfer data size (uint8_ts)

	};
	
	//	taskEXIT_CRITICAL();

	twi_master_write(TWI0, &packet_write);
	taskEXIT_CRITICAL();
	return 0;
}
