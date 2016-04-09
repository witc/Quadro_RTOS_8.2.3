/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       sx1276-Hal.c
 * \brief      SX1276 Hardware Abstraction Layer
 *
 * \version    2.0.B2 
 * \date       Nov 21 2012
 * \author     Miguel Luis
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */
#include <stdint.h>
#include <conf_board.h>
#include <stdbool.h> 
#include "FreeRTOS_V_8_Header.h"


//#include "ioe.h"
//#include "spi.h"
#include "board.h"
#include "sx1276-Hal.h"
#include <asf.h>
#include <ioport.h>


uint8_t SpiInOut(uint8_t Data);

#define ioport_set_pin_peripheral_mode(pin, mode) \
do {\
	ioport_set_pin_mode(pin, mode);\
	ioport_disable_pin(pin);\
} while (0)



extern struct spi_device device_rf;


void SX1276InitIo( void )
{
	
	ioport_set_pin_dir(SX1276_RxTx_PIN,  IOPORT_DIR_OUTPUT );
	ioport_set_pin_level(SX1276_RxTx_PIN,true);
	
	ioport_set_pin_dir(SX1276_RESET_PIN, IOPORT_DIR_OUTPUT );
	ioport_set_pin_level(SX1276_RESET_PIN,true);
	
		
	
}

void SX1276SetReset( uint8_t state )
{
	if( state == RADIO_RESET_ON )
	{
		// Set RESET pin to 1
		ioport_set_pin_level(SX1276_RESET_PIN, true);
		//ioport_set_pin_mode(SX1276_RESET_PIN, IOPORT_DIR_INPUT);
	}
	else
	{
		// Set RESET pin to 0
		ioport_set_pin_level(SX1276_RESET_PIN, false);
	}
}

void SX1276Write( uint8_t addr, uint8_t data )
{
    SX1276WriteBuffer( addr, &data, 1 );
}

void SX1276Read( uint8_t addr, uint8_t *data )
{
    SX1276ReadBuffer( addr, data, 1 );
}

// void SPI_Read_Send(uint8_t *pb_TxBuffer, uint8_t *pb_RxBuffer, uint8_t bLength )
// {
// 	uint8_t *m_pbRxBuffer = pb_RxBuffer;
// 	uint8_t *m_pbTxBuffer = pb_TxBuffer;
// 	uint8_t m_bLengthOfBytes = bLength;
// 	uint32_t ss = 0;
// 	// clear SS to LOW - ACTIVE
// 	spi_select_device(SPI, &ss);
// 	for (uint8_t i=0; i < m_bLengthOfBytes; i++)
// 	{
// 		//SPDR = m_pbTxBuffer[i];
// 		spi_put(SPI, m_pbTxBuffer[i]);
// 		//while(!(SPSR & (1<<SPIF)));
// 		while(!(spi_is_rx_ready(SPI)));
// 		//m_pbRxBuffer[i] = SPDR;
// 		m_pbRxBuffer[i] = spi_get(SPI);
// 	}
// 	// set SS to HIGH (INACTIVE)
// 	spi_deselect_device(SPI, &ss);
// 	return;
// }

void SX1276WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
     
//	taskENTER_CRITICAL();
	//spi_set_peripheral_chip_select_value(SPI, 1);	//pridano
	
	spi_select_device(SPI, &device_rf);
	//ioport_set_pin_level(SX1276_CS_PIN, false);
	spi_put(SPI, addr | 0x80);

	spi_write_packet(SPI,buffer,size);
	
	spi_deselect_device(SPI, &device_rf);
//	ioport_set_pin_level(SX1276_CS_PIN, true);
 	//taskEXIT_CRITICAL();
		

}

void SX1276ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
   
 //	taskENTER_CRITICAL();
	//spi_set_peripheral_chip_select_value(SPI, 1);	//pridano
    
	//ioport_set_pin_level(SX1276_CS_PIN, false);
	spi_select_device(SPI, &device_rf);
	
	spi_put(SPI, addr & 0x7F);
	spi_read_packet(SPI,buffer,size);

	//ioport_set_pin_level(SX1276_CS_PIN, true);
	spi_deselect_device(SPI, &device_rf);
	
 //	taskEXIT_CRITICAL();
	
}

void SX1276WriteFifo( uint8_t *buffer, uint8_t size )
{
    SX1276WriteBuffer( 0, buffer, size );
}

void SX1276ReadFifo( uint8_t *buffer, uint8_t size )
{
    SX1276ReadBuffer( 0, buffer, size );
}



uint8_t SX1276ReadDio0( void )
{
	//return (uint8_t)ioport_get_pin_level(SX1276_NIRQ0_PIN);
	return 0;
}

uint8_t SX1276ReadDio1( void )
{
	// Unfortunately we have not the free pin
	//return (uint8_t)ioport_get_pin_level(Sx1276_DI1_PIN);
	return(0);
}

uint8_t SX1276ReadDio2( void )
{
    //return GPIO_ReadInputDataBit( DIO2_IOPORT, DIO2_PIN );
	return(0);
}

uint8_t SX1276ReadDio3( void )
{
    //return IoePinGet( RF_DIO3_PIN );
	return(0);
}

uint8_t SX1276ReadDio4( void )
{
   // return IoePinGet( RF_DIO4_PIN );
   return(0);
}

uint8_t SX1276ReadDio5( void )
{
    //return IoePinGet( RF_DIO5_PIN );
	return(0);
}


void SX1276WriteRxTx( uint8_t txEnable )
{
    if( txEnable != 0 )
    {	
		// High in TX
		ioport_set_pin_level(SX1276_RxTx_PIN, true);
		
    }
    else
    {
		ioport_set_pin_level(SX1276_RxTx_PIN, false);
		
    }
}

