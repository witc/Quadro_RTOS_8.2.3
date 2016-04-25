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

uint32_t MPU_9150_send(unsigned char Adress, unsigned char *Data, unsigned char Length)
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

uint32_t MPU_9150_read(unsigned char Adress, unsigned char *Data, short Length)
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

void MPU_9150_init(void)
{
	
}