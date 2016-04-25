/*
 * Periph_init.c
 *
 * Created: 28.03.2016 21:48:03
 *  Author: J
 */ 
#include <asf.h>
#include "FreeRTOS_V_8_Header.h"
#include "RF_Task.h"
#include "Senzor_Task.h"
#include "Periph_init.h"

struct spi_device device_rf = {
	.id = 0,
};

struct spi_device device_mpu = {
	.id = 1,
};

struct spi_device adns_3080 = {
	.id = 2,
};




void interrupt_init(void)
{
	/* MPU 9150 */
	//pio_set_input(PIOB, MPU9150_PIN_INT, PIO_DEFAULT);
	pio_handler_set(PIOB, ID_PIOB, PIO_PB0, PIO_IT_RISE_EDGE, MPU9150_INT);
	pio_handler_set_priority(PIOB, PIOA_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	pio_enable_interrupt(PIOB, PIO_PB0);
		
	NVIC_DisableIRQ((IRQn_Type)PIOB_IRQn);
	NVIC_ClearPendingIRQ((IRQn_Type) PIOB_IRQn);
	NVIC_SetPriority((IRQn_Type) PIOB_IRQn,configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ((IRQn_Type) PIOB_IRQn);
	

//pio_handler_set_priority(PIOB, PIOB_IRQn, 0);
}

void bus_init(void)
{
	
	/* TWI init - MPU*/
	twi_master_options_t opt = {
		.speed = TWI_SPEED,
		.chip  = 0x19,
	};
	
	twi_master_setup(TWI0, &opt);
	twi_master_init(TWI0, &opt);
	twi_master_enable(TWI0);
	
	/* SPI - Semtech & MPU9250 */
	spi_flags_t spi_flags = SPI_MODE_0;
	spi_master_init(SPI);
	//	spi_set_master_mode(CONF_ILI9341_SPI);
	spi_master_setup_device(SPI, &device_rf, spi_flags,	CONF_ILI9341_CLOCK_SPEED*10, 0);
	spi_master_setup_device(SPI, &device_mpu, spi_flags,	CONF_ILI9341_CLOCK_SPEED, 1);	//mpu spi
	spi_master_setup_device(SPI, &adns_3080, spi_flags,	CONF_ILI9341_CLOCK_SPEED/2, 2);	//mpu spi
	//spi_set_peripheral_chip_select_value(CONF_ILI9341_SPI, 1);
	spi_enable(SPI);
	
	/* usart init - GPS*/
	const usart_serial_options_t usart_serial_options = {
		.baudrate   = 9600,
		.charlength = US_MR_CHRL_8_BIT,
		.paritytype = US_MR_PAR_NO,
		.stopbits   = false, /* No extra stopbit */
	};
		
	usart_reset_status(GPS_USART);
	usart_serial_init(GPS_USART,&usart_serial_options);
	//usart_set_rx_timeout(CONF_UART,10);
	//usart_enable_tx(CONF_UART);
	usart_enable_rx(GPS_USART);
	usart_enable_tx(GPS_USART);
	usart_get_status(GPS_USART);

}

void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = 115200,
		.charlength = 8,
		.paritytype = US_MR_PAR_NO,
		.stopbits = 1,

	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(ID_UART1);
	pio_configure_pin_group(PIOB, (PIO_PB2A_URXD1 | PIO_PB3A_UTXD1),
	(PIO_PERIPH_A | PIO_DEFAULT));
	stdio_serial_init(UART1, &uart_serial_options);
	
	//usart_write_line((Usart*)UART1,"**----------Start-----------**\n");
	
}