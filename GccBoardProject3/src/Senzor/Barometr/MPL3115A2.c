/*
 * MPL3115A2.c
 *
 * Created: 19.04.2016 16:29:39
 *  Author: J
 */ 
#include <asf.h>
#include <math.h>
#include <arm_math.h>
#include "FreeRTOS.h"
#include "main.h"
#include "Senzor_Task.h"
#include "FreeRTOS_V_8_Header.h"
#include "MPL3115A2.h"


uint8_t Baro_send(unsigned char Adress, unsigned char *Data, unsigned char Length)
{
	taskENTER_CRITICAL();

	twi_package_t packet_write = {
		.chip         =  MPL3115A2_ADDRESS,                        // TWI slave bus address
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


uint8_t Baro_read(unsigned char Adress, unsigned char *Data, unsigned char Length)
{
	taskENTER_CRITICAL();

	twi_package_t packet_read = {
		.chip         =  MPL3115A2_ADDRESS,                        // TWI slave bus address
		.addr         = Adress,								// TWI slave memory address data
		.addr_length  = sizeof (unsigned char),                    // TWI slave memory address data size
		.buffer       = Data,                        // transfer data destination buffer
		.length       = Length  // transfer data size (uint8_ts)

	};
	// Perform a multi-uint8_t read access then check the result.
	 //
	 twi_master_read(TWI0, &packet_read);
	
	taskEXIT_CRITICAL();
	
	return 0;

}


void Baro_init(void)
{	
	uint8_t temp=0;
	
	/*READ ID */
	Baro_read(MPL3115A2_WHOAMI,&temp,1);
	if (temp==0xC4)
	{ 
		/*ID reading ok*/
	}
	else
	{
		NVIC_SystemReset();
	}
	
	
	/* dta rate, ALT vs BARO mode */
	temp= MPL3115A2_CTRL_REG1_SBYB |MPL3115A2_CTRL_REG1_OS128 |MPL3115A2_CTRL_REG1_ALT;//
	Baro_send(MPL3115A2_CTRL_REG1,&temp,1);
	
	/* flags */
	temp=	MPL3115A2_PT_DATA_CFG_TDEFE |	MPL3115A2_PT_DATA_CFG_PDEFE |MPL3115A2_PT_DATA_CFG_DREM;//
	Baro_send(MPL3115A2_PT_DATA_CFG,&temp,1);
}

uint8_t Baro_get_preasure(uint8_t* data)
{	
	uint8_t temp;
	
	Baro_read(MPL3115A2_REGISTER_STATUS,&temp,1);
	
	if ((temp&0x80)!=0x80)
	{	
		toggleOneShot(); //Toggle the OST bit causing the sensor to immediately take another reading
		return 1;
	}
		 
	else
	{
		Baro_read(MPL3115A2_REGISTER_PRESSURE_MSB,data,3);
		toggleOneShot(); //Toggle the OST bit causing the sensor to immediately take another reading
		return 0;
	}
	
}


//Call with a rate from 0 to 7. See page 33 for table of ratios.
//Sets the over sample rate. Datasheet calls for 128 but you can set it 
//from 1 to 128 samples. The higher the oversample rate the greater
//the time between data samples.
void setOversampleRate(uint8_t sampleRate)
{
  if(sampleRate > 7) sampleRate = 7; //OS cannot be larger than 0b.0111
  sampleRate <<= 3; //Align it for the CTRL_REG1 register
  
  uint8_t tempSetting = Baro_read(CTRL_REG1,&tempSetting,1); //Read current settings
  tempSetting &= 0b11000111; //Clear out old OS bits
  tempSetting |= sampleRate; //Mask in new OS bits
  Baro_send(CTRL_REG1, &tempSetting,1);
}



//Clears then sets the OST bit which causes the sensor to immediately take another reading
//Needed to sample faster than 1Hz
void toggleOneShot(void)
{
  uint8_t tempSetting ;
  Baro_read(CTRL_REG1,&tempSetting,1); //Read current settings
  tempSetting &= ~(1<<1); //Clear OST bit
  Baro_send(CTRL_REG1, &tempSetting,1);

  tempSetting = Baro_read(CTRL_REG1,&tempSetting,1); //Read current settings to be safe
  tempSetting |= (1<<1); //Set OST bit
  Baro_send(CTRL_REG1, &tempSetting,1);
}


void Baro_Calibrate( float *init_altitude )
{
	
 	uint8_t tlak[3];
 	float temp=0;
// 	
 	while(Baro_get_preasure(tlak)==1);
 	while(Baro_get_preasure(tlak)==1);
 	while(Baro_get_preasure(tlak)==1);
// 	
	for (uint8_t a=0;a<10;a++)
	{
		while(Baro_get_preasure(tlak)==1);
		temp+= (float) ((short) ((tlak[0] << 8) | tlak[1])) + (float) (tlak[2] >> 4) * 0.0625;
		vTaskDelay(50/portTICK_RATE_MS);
	}
	
	*init_altitude=(float)(temp/10);
	
	// Reset Altitude Offset  = 0
// 	unsigned char barobuff = 0;
// 	Baro_send(OFF_P, &barobuff,  1 );
// 
// 	// Measure current pressure
// 	// -> Set back To BAROMETER mode with OSR = 128
// 	barobuff = 0x39;
// 	Baro_send( MPL3115A2_CTRL_REG1, &barobuff, 1 );
// 	
// 	// Read current pressure
// 	uint8_t data[3];
// 	while(Baro_get_preasure(data)==1);
// 	double current_pressure = (double) ((short) ((data[0] << 8) | data[1])) + (double) (data[2] >> 4) * 0.0625;
// 
// 	// Calculate Pressure at mean sea level based on a given altitude:
// 	double sea_pressure = current_pressure * pow( 1-352*0.0000225577, 5.255877 );
// 
// 	// Send this value now back to the sensor:
// 	unsigned char buffer[2];
// 	buffer[0] = (unsigned int)(sea_pressure / 2)>>8;
// 	buffer[1] = (unsigned int)(sea_pressure / 2)&0xFF;
// 	Baro_send( BAR_IN_MSB, buffer, 2 );

	vTaskDelay(100/portTICK_RATE_MS);

}