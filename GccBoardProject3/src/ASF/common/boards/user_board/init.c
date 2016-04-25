/**
 * \file
 *
 * \brief User board initialization template
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>
#include "FreeRTOS_V_8_Header.h"
#include "RF_Task.h"

void board_init(void)
{
	ioport_init();
	
	/* SPI pins set */
	gpio_configure_pin(SPI_MISO_GPIO, SPI_MISO_FLAGS);
	gpio_configure_pin(SPI_MOSI_GPIO, SPI_MOSI_FLAGS);
	gpio_configure_pin(SPI_SPCK_GPIO, SPI_SPCK_FLAGS);
	
	/* SEMTECH */
	gpio_configure_pin(SX1276_CS_PIN, SPI_NPCS0_FLAGS);
	ioport_set_pin_level(SX1276_CS_PIN,true);
	
	/* ADNS 3080*/
	gpio_configure_pin(ADNS_CS_PIN, SPI_NPCS2_PA10_FLAGS);
	//ioport_set_pin_dir(ADNS_CS_PIN,IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(ADNS_CS_PIN,true);
	ioport_set_pin_dir(ADNS_RESET_PIN,IOPORT_DIR_OUTPUT);
	
	
	/* MPU9250 */
	gpio_configure_pin(MPU9150_CS_PIN, SPI_NPCS2_PA10_FLAGS);
	ioport_set_pin_level(MPU9150_CS_PIN,true);
			
	/* Measure pins */
	ioport_set_pin_dir(PERIODE_PIN,1);
	ioport_set_pin_dir(PERIODE_PIN_INT,1);
		
	/* LED output */
	ioport_set_pin_dir(LEDW,IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(LEDR,IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(LED0,IOPORT_DIR_OUTPUT);
	
	//GPS
	gpio_configure_pin(GPS_TX_PIN, PIO_PERIPH_A | PIO_DEFAULT);
	gpio_configure_pin(GPS_RX_PIN, PIO_PERIPH_A | PIO_DEFAULT);
	ioport_set_pin_dir(GPS_RESET_PIN,IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(GPS_RESET_PIN,true);
	
	//Baro
	ioport_set_pin_dir(I2C_SCL_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(I2C_SDA_PIN, IOPORT_DIR_OUTPUT);
	gpio_configure_pin(I2C_SDA_PIN,TWI0_DATA_FLAGS);
	gpio_configure_pin(I2C_SCL_PIN,TWI0_CLK_FLAGS);
	
	
}
