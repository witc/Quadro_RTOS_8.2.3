/*
 * ADNS_3080_HALc.c
 *
 * Created: 15.04.2016 22:13:46
 *  Author: J
 */ 

#include <asf.h>
#include "FreeRTOS_V_8_Header.h"
#include "Periph_init.h"
#include "ADNS_3080-HAL.h"

extern struct spi_device adns_3080;


uint32_t ADNS_3080_send(unsigned char Adress, unsigned char *Data, unsigned char Length)
{
	taskENTER_CRITICAL();
	uint8_t status=0;
	spi_select_device(SPI, &adns_3080);
	//ioport_set_pin_level(SX1276_CS_PIN, false);
	spi_put(SPI, Adress & 0x7F| 0x80);
	status=spi_write_packet(SPI,Data,Length);
	
	spi_deselect_device(SPI, &adns_3080);
	taskEXIT_CRITICAL();
	
	return status;
	
}

/**************************************/

uint32_t ADNS_3080_read(unsigned char Adress, unsigned char *Data, short Length)
{
	uint8_t status=0;
	taskENTER_CRITICAL();
	
	spi_select_device(SPI, &adns_3080);
	spi_put(SPI, Adress & 0x7F );
	status=spi_read_packet(SPI,Data,Length);

	//ioport_set_pin_level(SX1276_CS_PIN, true);
	spi_deselect_device(SPI, &adns_3080);
	taskEXIT_CRITICAL();
	return status;
	
}
