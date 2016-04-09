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
	
	gpio_configure_pin(SX1276_CS_PIN, SPI_NPCS0_FLAGS);
	ioport_set_pin_level(SX1276_CS_PIN,true);
	
	/*MPU9255 pins init*/
	//Compass
 	ioport_set_pin_dir(I2C_SCL_PIN, IOPORT_DIR_OUTPUT);
 	ioport_set_pin_dir(I2C_SDA_PIN, IOPORT_DIR_OUTPUT);
	
	ioport_set_pin_dir(PIO_PC24_IDX, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIO_PC24_IDX,false);
	ioport_set_pin_level(I2C_SCL_PIN,false);
	ioport_set_pin_level(I2C_SDA_PIN,false);
	delay_ms(250);
	ioport_set_pin_level(PIO_PC24_IDX,true);
// 	ioport_set_pin_level(I2C_SCL_PIN,false);
// 	ioport_set_pin_level(I2C_SDA_PIN,false);
	delay_ms(100);
	gpio_configure_pin(I2C_SCL_PIN, TWI0_CLK_FLAGS);
	gpio_configure_pin(I2C_SDA_PIN, TWI0_DATA_FLAGS);
	
	/* Measure pins */
	ioport_set_pin_dir(PERIODE_PIN,1);
	ioport_set_pin_dir(PERIODE_PIN_INT,1);
	
	
	/* LED output */
	ioport_set_pin_dir(LEDW,IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(LEDR,IOPORT_DIR_OUTPUT);
	
	/*int init - semtech*/
	//pmc_enable_periph_clk(ID_PIOA);
	
	//pio_set_input(PIOA, PIO_PA17, PIO_PULLUP);
	//pio_set_input(PIOA, PIO_PA18, PIO_PULLUP);
	//pio_set_input(PIOA, PIO_PA24, PIO_PULLUP);
	
	
	
	
}
