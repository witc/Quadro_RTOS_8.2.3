/*
 * MPU_9150.c
 *
 * Created: 02.12.2015 21:57:32
 *  Author: uzivatel
 */ 

#include <asf.h>
#include "main.h"
#include "MPU_9150.h"
#include "MPU_9150_HAL.h"
#include "FreeRTOS_V_8_Header.h"

Pdc *twi_dma_inst;
pdc_packet_t twi_dma_packet;


/* Hardware registers needed by driver. */
struct gyro_reg_s {
	unsigned char who_am_i;
	unsigned char rate_div;
	unsigned char lpf;
	unsigned char prod_id;
	unsigned char user_ctrl;
	unsigned char fifo_en;
	unsigned char gyro_cfg;
	unsigned char accel_cfg;
	unsigned char accel_cfg2;
	unsigned char lp_accel_odr;
	unsigned char motion_thr;
	unsigned char motion_dur;
	unsigned char fifo_count_h;
	unsigned char fifo_r_w;
	unsigned char raw_gyro;
	unsigned char raw_accel;
	unsigned char temp;
	unsigned char int_enable;
	unsigned char dmp_int_status;
	unsigned char int_status;
	unsigned char accel_intel;
	unsigned char pwr_mgmt_1;
	unsigned char pwr_mgmt_2;
	unsigned char int_pin_cfg;
	unsigned char mem_r_w;
	unsigned char accel_offs;
	unsigned char i2c_mst;
	unsigned char bank_sel;
	unsigned char mem_start_addr;
	unsigned char prgm_start_h;
	#if defined AK89xx_SECONDARY
	unsigned char s0_addr;
	unsigned char s0_reg;
	unsigned char s0_ctrl;
	unsigned char s1_addr;
	unsigned char s1_reg;
	unsigned char s1_ctrl;
	unsigned char s4_ctrl;
	unsigned char s0_do;
	unsigned char s1_do;
	unsigned char i2c_delay_ctrl;
	unsigned char raw_compass;
	/* The I2C_MST_VDDIO bit is in this register. */
	unsigned char yg_offs_tc;
	#endif
};

//extern vSemaphoreCreateBinary()

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 4g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void MPU6050_Initialize(void)
{	
	
	unsigned char data[6];
	
	/* Wake up chip. */
	data[0] = 0x00;
	if(MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_PWR_MGMT_1 )==TWI_SUCCESS)
	{
		usart_write_line((Usart*)UART1,"Wake UP MPU chip OK...\n");
	}
	else
	{
		usart_write_line((Usart*)UART1,"Wake UP MPU FALSE...\n");
		NVIC_SystemReset();
	}
	
	vTaskDelay(10/portTICK_RATE_MS);	
    /* Reset device. */
    data[0] = BIT_RESET;
	if(MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_PWR_MGMT_1 )==TWI_SUCCESS)
	{
		usart_write_line((Usart*)UART1,"MPU - RESET OK...\n");			 
	}else
	{
		usart_write_line((Usart*)UART1,"MPU - RESET FALSE...\n");
		NVIC_SystemReset();		
	}
	
	vTaskDelay(100/portTICK_RATE_MS);
	/* Wake up chip. */
	data[0] = 0x00;
	if(MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_PWR_MGMT_1 )==TWI_SUCCESS)
	{
		usart_write_line((Usart*)UART1,"Wake UP MPU chip OK...\n");		
	}else
	{
		usart_write_line((Usart*)UART1,"Wake UP MPU FALSE...\n");
		NVIC_SystemReset();
	}

 	if(MPU6050_TestConnection()==true)
 	{
 		usart_write_line((Usart*)UART1,"Test MPU ID OK...\n");		
 	}else
	{
 		usart_write_line((Usart*)UART1,"Test MPU ID False...\n");
 		NVIC_SystemReset();
	}

	
	/* Wake up chip. */
	data[0] = 0x00;
	if(MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_PWR_MGMT_1 )==TWI_SUCCESS)
	{
		usart_write_line((Usart*)UART1,"Wake UP MPU chip OK...\n");
	}
	else
	{
		usart_write_line((Usart*)UART1,"Wake UP MPU FALSE...\n");
		NVIC_SystemReset();
	}
	
	
	MPU6050_SetClockSource(MPU6050_CLOCK_PLL_XGYRO );	//
	 
	data[0] = 0x00;
	MPU6050_I2C_ByteWrite(10,data, MPU6050_RA_INT_ENABLE);   // Disable all interrupts
	MPU6050_I2C_ByteWrite(10,data, MPU6050_RA_FIFO_EN);      // Disable FIFO
	MPU6050_I2C_ByteWrite(10,data, MPU6050_RA_PWR_MGMT_1);   // Turn on internal clock source
	MPU6050_I2C_ByteWrite(10,data, MPU6050_RA_I2C_MST_CTRL); // Disable I2C master
	MPU6050_I2C_ByteWrite(10,data, MPU6050_RA_USER_CTRL);    // Disable FIFO and I2C master modes
	data[0] = 0x0C;
	MPU6050_I2C_ByteWrite(10,data, MPU6050_RA_USER_CTRL);    // Reset FIFO and DMP

	/*set LPF and Fs to 8khz - internal */
	data[0]=3;	/* 3=42 hz - used in Afroflight, 4=21Hz cut off -> for 0 & 7 = off*/
	MPU6050_I2C_ByteWrite(10,&data[0], MPU6050_RA_CONFIG);

	/* set sample rate */
	data[0]=1;	// 1 = 500HZ
	MPU6050_I2C_ByteWrite(10,data,MPU6050_RA_SMPLRT_DIV); // 1khz / (1 + 3) =250hz

	/* Set citlivost */
	MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_2000);
	// 	 data[0]=0x14;
	// 	 MPU6050_I2C_ByteWrite(10,data,0x1B);

	MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_4);

#if (FIFO_MPU9150==1)
	/* Fifo enable + FIFO reset */
	data[0] =0x44;
	MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_USER_CTRL );

	/* Enable Fifo, GYRO, Temp and Acc*/
	data[0]=0x70;
	MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_FIFO_EN);
#endif

	/* Data ready interrupts enabled*/

#if (RAW_MPU9150==1)
	 
#elif (RAW_INT_MPU9150==1)
	data[0]=0x1;//INT DRDY

#elif (FIFO_MPU9150==1)
	data[0]=0x10; // Fifo owerflow - no needed

#else
	# error "Please specifyWay to get a datta from MPU9150"
#endif
	
	vTaskDelay(50/portTICK_RATE_MS);
	if(MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_INT_ENABLE)==TWI_SUCCESS)
	{
		usart_write_line((Usart*)UART1,"INT enabled MPU OK...\n");		
	}
	else
	{
		usart_write_line((Usart*)UART1,"INT enabled MPU FALSE...\n");
		NVIC_SystemReset();
		
	}

	
	

}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, FALSE otherwise
 */
bool MPU6050_TestConnection(void)
{
	return MPU6050_GetDeviceID() == 0x34 ? true : false; //0b110100; 8-bit representation in hex = 0x34
}

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100).
 * @return Device ID (should be 0x68, 104 dec, 150 oct), but after masking 0x34
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
uint8_t MPU6050_GetDeviceID(void)
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &tmp);
    return tmp;
}


/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see MPU6050_GetClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void MPU6050_SetClockSource(uint8_t source)
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see MPU6050_GetFullScaleGyroRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_SetFullScaleGyroRange(uint8_t range)
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
uint8_t MPU6050_GetFullScaleGyroRange(void)
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, &tmp);
    return tmp;
}

/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
uint8_t MPU6050_GetFullScaleAccelRange(void)
{
    uint8_t tmp;
    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, &tmp);
    return tmp;
}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see MPU6050_GetFullScaleAccelRange()
 */
void MPU6050_SetFullScaleAccelRange(uint8_t range)
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
bool MPU6050_GetSleepModeStatus(void)
{
    uint8_t tmp;
    MPU6050_ReadBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, &tmp);
    return tmp == 0x00 ? false : true;
}

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see MPU6050_GetSleepModeStatus()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void MPU6050_SetSleepModeStatus(uint8_t NewState)
{
    MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, NewState);
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param AccelGyro 16-bit signed integer array of length 6
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
void MPU6050_GetRawAccelGyro(short* AccelGyro)
{
    uint8_t tmpBuffer[14];
    MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, tmpBuffer, MPU6050_RA_ACCEL_XOUT_H, 14);
    /* Get acceleration */
    for (int i = 0; i < 3; i++)
        AccelGyro[i] = ((short) ((uint16_t) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);
    /* Get Angular rate */
    for (int i = 4; i < 7; i++)
        AccelGyro[i - 1] = ((short) ((uint16_t) tmpBuffer[2 * i] << 8) + tmpBuffer[2 * i + 1]);

}

/** Write multiple bits in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 */
void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data; // combine data with existing byte
    MPU6050_I2C_ByteWrite(slaveAddr, &tmp, regAddr);
}

/** write a single bit in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 */
void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    MPU6050_I2C_ByteWrite(slaveAddr, &tmp, regAddr);
}

/** Read multiple bits from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
     tmp &= mask;
     tmp >>= (bitStart - length + 1);
    *data = tmp;
}

/** Read a single bit from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
    uint8_t tmp;
    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    *data = tmp & (1 << bitNum);
}



/**
 * @brief  Writes one byte to the  MPU6050.
 * @param  slaveAddr : slave address MPU6050_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer  containing the data to be written to the MPU6050.
 * @param  writeAddr : address of the register in which the data will be written
 * @return None
 */
uint32_t MPU6050_I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t writeAddr)
{
    // ENTR_CRT_SECTION();
	
	return MPU_9150_send(writeAddr,pBuffer,1);

}

/**
 * @brief  Reads a block of data from the MPU6050.
 * @param  slaveAddr  : slave address MPU6050_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
 * @param  readAddr : MPU6050's internal address to read from.
 * @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
 * @return None
 */
uint32_t MPU6050_I2C_BufferRead(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t readAddr, short NumByteToRead)
{
    // ENTR_CRT_SECTION();
 //	for (short i=0;i<NumByteToRead;i++)
 //	{
		return MPU_9150_read(readAddr,&pBuffer[0],NumByteToRead);	
		    
}
/**
 * @}
 *//* end of group MPU6050_Library */
 
 
void MPU9150_getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz)
 {

    //get accel and gyro
    //MPU9150_getMotion6(ax, ay, az, gx, gy, gz);

    //read mag
//     MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_PIN_CFG, 0x02); //set i2c bypass enable pin to true to access magnetometer
//    // delay_ms(10);
//     MPU6050_I2C_ByteWrite(MPU6050_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
//      //delay_ms(10);
//      MPU6050_I2C_BufferRead(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer);
//     *mx = (((int16_t)buffer[0]) << 8) | buffer[1];
//     *my = (((int16_t)buffer[2]) << 8) | buffer[3];
//     *mz = (((int16_t)buffer[4]) << 8) | buffer[5];
}
/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU9150_RA_ACCEL_XOUT_H
 */
void MPU9150_getMotion6(short* ax, short* ay, short* az, short* gx, short* gy, short* gz,short *offset) 
{	
	uint8_t buffer[14];
    MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,buffer, MPU6050_RA_ACCEL_XOUT_H, 12);
    *ax = (((short)buffer[0]) << 8) | buffer[1];
    *ay = (((short)buffer[2]) << 8) | buffer[3];
    *az = (((short)buffer[4]) << 8) | buffer[5];
    *gx = ((((short)buffer[8]) << 8) | buffer[9])-offset[0];
    *gy = ((((short)buffer[10]) << 8) | buffer[11])-offset[1];
    *gz = ((((short)buffer[12]) << 8) | buffer[13])-offset[2];
}

void MPU9150_getMotion3(uint8_t *buffer)
{
	
	MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,buffer,MPU6050_RA_GYRO_XOUT_H, 6);
	
}
/* not fifo, but clear data */
void MPU9150_getMotion_dma(uint8_t *buffer)
{	
// 	uint8_t temp_addres=MPU6050_RA_GYRO_XOUT_H;
// 	twi_dma_inst=twi_get_pdc_base(TWI0);
// 	// TWI0 receive is channel 8
// 	sysclk_enable_peripheral_clock(twi_dma_inst);
// 	
// 	/* Initialize PDC data packet for transfer */
// 	twi_dma_packet.ul_addr = (uint32_t) temp_addres;
// 	twi_dma_packet.ul_size = 1;
// 	/* Configure PDC for data receive */
// 	pdc_tx_init(twi_dma_inst, &twi_dma_packet, NULL);
// 	/* Enable PDC transfers */
// 	pdc_enable_transfer(twi_dma_inst,  PERIPH_PTCR_TXTEN);
// 		
	
	//MPU6050_I2C_ByteWrite() 		
	/*twi int enable*/
	twi_enable_interrupt(TWI0, TWI_IER_TXCOMP);	//tx completed
	
	NVIC_SetPriority(TWI0_IRQn,configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(TWI0_IRQn);
	
}
/* FIFO DATA */
short MPU9150_getMotion_fifo(uint8_t* FIFO_MPU)
{	
	
	short temp;
	uint8_t data[3];
		
// 	 	/* disable write to fifo  */
// 	    	data[0]=0;
// 	    	MPU6050_I2C_ByteWrite(10,data,MPU6050_RA_FIFO_EN);
			
	/* read count of Fifo */
	MPU6050_I2C_BufferRead(10,data,MPU6050_RA_FIFO_COUNTH,2);
	temp=(((short)data[0]) << 8) | data[1];
	//temp-=6*25;	//budeme èíst o x Bytes ménì
		
	/* read FIFO */
	if (temp>=14)
	{
		MPU6050_I2C_BufferRead(10,&FIFO_MPU[0],MPU6050_RA_FIFO_R_W,temp);	
	}
	return temp;	
// 		 	/* Enable Fifo, GYRO, Temp and Acc*/
// 		   	data[0]=0x70;
// 		  	MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_FIFO_EN);


// 	/* disable write to fifo  */
//    	data[0]=0;
//    	MPU6050_I2C_ByteWrite(10,data,MPU6050_RA_FIFO_EN);
// 	
// 	/* read count of Fifo */
// 	MPU6050_I2C_BufferRead(10,data,MPU6050_RA_FIFO_COUNTH,2);
// 	temp=(((short)data[0]) << 8) | data[1];
 	 
// 	 /* read FIFO */
// 	MPU6050_I2C_BufferRead(10,&FIFO_MPU[0],MPU6050_RA_FIFO_R_W,temp);
// 
// 	/* Enable Fifo, GYRO, Temp and Acc*/
//   	data[0]=0x70;
//  	MPU6050_I2C_ByteWrite(10,&data[0],MPU6050_RA_FIFO_EN);


	
}


/************************************************************************/
/* Compute Tempreature bias Gyro                                        */
/************************************************************************/
void MPU9150_Gyro_Tempr_Bias(short *offset)
{
	 long Sum[4]={0,0,0,0};	//32 bits
	static short Temp;
	//static short Temperature;
	//static short Pre_Temp=0;
	uint8_t *p_buffer;	//pointer for malloc buffer
	#define NO_OF_PERIOD 20
	uint16_t count=0;
	uint16_t packet_count=0;
	uint16_t count_sum=0;
	
	/* alocate buffer for reading fifo */
	p_buffer=pvPortMalloc(sizeof(uint8_t)*1024);
	
	vTaskDelay(150/portTICK_RATE_MS);

	count=MPU9150_getMotion_fifo(&p_buffer[0]);
	
	for (short i=0;i<NO_OF_PERIOD;i++)
	{	
		vTaskDelay(50/portTICK_RATE_MS);

		ioport_set_pin_level(PERIODE_PIN,true);
	
		count=MPU9150_getMotion_fifo(&p_buffer[0]);
		
		  packet_count = 0;
		  
		  while (count>0)
		  {	
			  count-=6;
			Temp=(short)((p_buffer[packet_count++] << 8) | p_buffer[packet_count++]);
			Sum[0]+=Temp;
			Temp=(short)((p_buffer[packet_count++] << 8) | p_buffer[packet_count++]);
			Sum[1]+=Temp;
			Temp=(short)((p_buffer[packet_count++] << 8) | p_buffer[packet_count++]);
			Sum[2]+=Temp;
			count_sum+=1;
		}
		
		ioport_set_pin_level(PERIODE_PIN,false);	
		
	}
	
	/* uvolneni buffru */
	 vPortFree(*p_buffer);
	 *p_buffer = NULL  ;
	 	
	 offset[0]=(short)(Sum[0]/count_sum);
	offset[1]=(short)(Sum[1]/count_sum);
	offset[2]=(short)(Sum[2]/count_sum)+1;

}
		 		

uint8_t MPU9150_Gyro_Tempr_Bias_no_fifo(short *offset)
{	
	static short Temp;
	uint8_t p_buffer[6];
	long long Sum[4]={0,0,0,0};	//32 bits
	#define NO_OF_SAMPLES 1000
	
	vTaskDelay(1000/portTICK_RATE_MS);
	for (short i=0;i<NO_OF_SAMPLES;i++)
	{
		MPU9150_getMotion3(p_buffer);
		Temp=(short)((p_buffer[0] << 8) | p_buffer[1]);
		Sum[0]+=Temp;
		Temp=(short)((p_buffer[2] << 8) | p_buffer[3]);
		Sum[1]+=Temp;
		Temp=(short)((p_buffer[4] << 8) | p_buffer[5]);
		Sum[2]+=Temp;
		vTaskDelay(1/portTICK_RATE_MS);

	}
	
	 offset[0]=(short)(Sum[0]/NO_OF_SAMPLES);
	 offset[1]=(short)(Sum[1]/NO_OF_SAMPLES);
	 offset[2]=(short)(Sum[2]/NO_OF_SAMPLES);
	 
	 return 0;
}

// 	for (short i=0;i<NO_OF_SAMPLES;i++)
// 	{
// 		MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,buffer, MPU6050_RA_GYRO_XOUT_H  , 6);  
// // 		  
// // 		Temp=(short)((buffer[0] << 8) | buffer[1]);
// // 		Sum[3]+=(float)Temp;
// 		Temp=(short)((buffer[0] << 8) | buffer[1]);
// 		Sum[0]+=(float)Temp;
// 		Temp=(short)((buffer[2] << 8) | buffer[3]);
// 		Sum[1]+=(float)Temp;
// 		Temp=(short)((buffer[4] << 8) | buffer[5]);
// 		Sum[2]+=(float)Temp;
// 		//delay_ms(2);
// 	}
//	/* Temp */
//	Temperature=(short)(((Sum[3]/NO_OF_SAMPLES)/340)+36.53);
	/*X,Y,Z gyro offset */
	
	
	
	/* waiting for incriasing tempreature */
	//delay_ms(10000);
		
// 	for (short i=0;i<NO_OF_SAMPLES;i++)
// 	{
// 		MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,buffer, MPU6050_RA_TEMP_OUT_H , 8);
// 		
// 		Temp=(short)((buffer[0] << 8) | buffer[1]);
// 		Sum[3]+=(float)Temp;
// 		Temp=(short)((buffer[2] << 8) | buffer[3]);
// 		Sum[0]+=(float)Temp;
// 		Temp=(short)((buffer[4] << 8) | buffer[5]);
// 		Sum[1]+=(float)Temp;
// 		Temp=(short)((buffer[6] << 8) | buffer[7]);
// 		Sum[2]+=(float)Temp;
// 		
// 	}
// 	
	/* Temp */
//	Temperature=(short)((Sum[3]/NO_OF_SAMPLES)/340+36.53);
// 	/*X,Y,Z gyro offset */
// 	offset[0]=(short)(Sum[0]/NO_OF_SAMPLES);
// 	offset[1]=(short)(Sum[1]/NO_OF_SAMPLES);
// 	offset[2]=(short)(Sum[2]/NO_OF_SAMPLES	
// 	/* Read actual temprreature */
// 	MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,buffer, MPU6050_RA_TEMP_OUT_H , 2);    
// 	Temp=(((short)buffer[0]) << 8) | buffer[1];
     


// setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!


