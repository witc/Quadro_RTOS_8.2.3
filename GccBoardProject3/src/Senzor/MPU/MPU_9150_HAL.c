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

struct spi_device device_mpu = {
	.id = 1,
};

uint32_t MPU_9150_send(unsigned char Adress, unsigned char *Data, unsigned char Length)
{
	//taskENTER_CRITICAL();
	
// 	spi_select_device(SPI, &device_mpu);
// 	ioport_set_pin_level(MPU9150_CS_PIN, false);
// 	spi_put(SPI, Adress | 0x80);
// 
// 	spi_write_packet(SPI,Data,Length);
// 	
// 	spi_deselect_device(SPI, &device_mpu);
	
	twi_package_t packet_write = {
		.chip         =  MPU6050_DEFAULT_ADDRESS,                        // TWI slave bus address
		.addr         = Adress,								// TWI slave memory address data
		.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
		.buffer       = Data,                        // transfer data destination buffer
		.length       = Length  // transfer data size (bytes)

	};
// 	
 	//while(twi_master_write(TWI0, &packet_write) != TWI_SUCCESS);
	return twi_master_write(TWI0, &packet_write);
//	taskEXIT_CRITICAL();

}
/**************************************/

uint32_t MPU_9150_read(unsigned char Adress, unsigned char *Data, short Length)
{	
	//taskENTER_CRITICAL();
	
// 	spi_select_device(SPI, &device_mpu);
// 	
// 	spi_put(SPI, Adress & 0x7F);
// 	spi_read_packet(SPI,Data,Length);
// 
// 	//ioport_set_pin_level(SX1276_CS_PIN, true);
// 	spi_deselect_device(SPI, &device_mpu);
	
	twi_package_t packet_read = {
		.chip         =  MPU6050_DEFAULT_ADDRESS,                        // TWI slave bus address
		.addr         = Adress,								// TWI slave memory address data
		.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
		.buffer       = Data,                        // transfer data destination buffer
		.length       = Length  // transfer data size (bytes)

	};
	// Perform a multi-byte read access then check the result.
	return twi_master_read(TWI0, &packet_read);
// 	twi_master_read(TWI0, &packet_read);
	//twi

	//taskEXIT_CRITICAL();
}

void MPU_9150_init(void)
{
	
}