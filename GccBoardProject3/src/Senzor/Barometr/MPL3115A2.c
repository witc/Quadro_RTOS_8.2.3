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
#include "MS5611.h"

#define QMF_SORT(a,b) { if ((a)>(b)) QMF_SWAP((a),(b)); }
#define QMF_SWAP(a,b) { float temp=(a);(a)=(b);(b)=temp; }
#define QMF_COPY(p,v,n) { int32_t i; for (i=0; i<n; i++) p[i]=v[i]; }
#define PRESSURE_SAMPLES_MEDIAN 3

 float applyBarometerMedianFilter(float newPressureReading)
{
	static float barometerFilterSamples[PRESSURE_SAMPLES_MEDIAN];
	static int currentFilterSampleIndex = 0;
	static bool medianFilterReady = false;
	int nextSampleIndex;
	
	nextSampleIndex = (currentFilterSampleIndex + 1);
	if (nextSampleIndex == PRESSURE_SAMPLES_MEDIAN) {
		nextSampleIndex = 0;
		medianFilterReady = true;
	}

	barometerFilterSamples[currentFilterSampleIndex] = newPressureReading;
	currentFilterSampleIndex = nextSampleIndex;
	
	if (medianFilterReady)
		return quickMedianFilter3(barometerFilterSamples);
	else
		return newPressureReading;

}

float quickMedianFilter3(float * v)
{
	float p[3];
	QMF_COPY(p, v, 3);

	QMF_SORT(p[0], p[1]); QMF_SORT(p[1], p[2]); QMF_SORT(p[0], p[1]) ;
	return p[1];
}


uint8_t Baro_send(unsigned char Adress, unsigned char *Data, unsigned char Length)
{
	taskENTER_CRITICAL();
	
	//uint8_t temp=0;
	//temp=*Data;
	
	twi_package_t packet_write = {
		.chip         =  MS5611_ADDR,                        // TWI slave bus address
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
	
	/*reset */
	temp= MPL3115A2_CTRL_REG1_RST;
	Baro_send(CTRL_REG1,&temp,1);
	vTaskDelay(10/portTICK_RATE_MS);
	
	/*READ ID */
	Baro_read(MPL3115A2_WHOAMI,&temp,1);
	if (temp==0xC4)
	{ 
		/*ID reading ok*/
	}
	else
	{
		//NVIC_SystemReset();
		return -1;
	}
	
	https://community.freescale.com/docs/DOC-95378
	/* dta rate, ALT vs BARO mode */
// 	temp= 0x07;//MPL3115A2_CTRL_REG1_SBYB |MPL3115A2_CTRL_REG1_OS128 |MPL3115A2_CTRL_REG1_BAR;//
// 	Baro_send(CTRL_REG1,&temp,1);	//data flags
// 	temp=0x11;
// 	Baro_send(CTRL_REG3,&temp,1);	//active low int, open drain
// 	temp=0x80;
// 	Baro_send(CTRL_REG4,&temp,1);	//drdy
// 	temp=0x00;
// 	Baro_send(CTRL_REG5,&temp,1);	  // DRDY interrupt routed to INT2 - PTD3
// 	temp=0xB9;
// 	Baro_send(CTRL_REG1,&temp,1);	// Active altimeter mode, OSR = 128

	MPL3115A2_setModeAltimeter();
	vTaskDelay(1/portTICK_RATE_MS);
	setOversampleRate(7);
	vTaskDelay(1/portTICK_RATE_MS);
	MPL3115A2_enableEventFlags();
	vTaskDelay(1/portTICK_RATE_MS);
	MPL3115A2_setModeActive();

	vTaskDelay(1/portTICK_RATE_MS);
	
// 	/* flags */
// 	temp=	MPL3115A2_PT_DATA_CFG_TDEFE |	MPL3115A2_PT_DATA_CFG_PDEFE |MPL3115A2_PT_DATA_CFG_DREM;//
// 	Baro_send(MPL3115A2_PT_DATA_CFG,&temp,1);
}

uint8_t Baro_get_preasure(uint8_t* data)
{	
	uint8_t temp=0;
	
	Baro_read(DR_STATUS,&temp,1);
	
	if ((temp&0x08)!=0x08)	//new data Preasure,/altuimeter are ava....
	{	
	//MPL3115A2_setModeActive();
		//MPL3115A2_setModeActive();
		toggleOneShot(); //Toggle the OST bit causing the sensor to immediately take another reading
		
		
		return 1;
	}
		 
	else
	{
				
		Baro_read(MPL3115A2_REGISTER_PRESSURE_MSB,data,5);
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
   uint8_t tempSetting=0 ;
   
   MPL3115A2_setModeActive();
   
  Baro_read(CTRL_REG1,&tempSetting,1); //Read current settings
  tempSetting &= ~(1<<1); //Clear OST bit
  Baro_send(CTRL_REG1, &tempSetting,1);
// 
  Baro_read(CTRL_REG1,&tempSetting,1); //Read current settings to be safe
  tempSetting |= (1<<1); //Set OST bit
  Baro_send(CTRL_REG1, &tempSetting,1);
   
// 	
// 	MPL3115A2_clearRegisterBit(CTRL_REG1, 0x02);  // Clear OST bit
//   	MPL3115A2_setRegisterBit(CTRL_REG1, 0x02);    // Set the OST bit.
  	 
}


void Baro_Calibrate( float *init_altitude )
{
	
 	uint8_t tlak[3];
 	float temp=0;
	// Reset Altitude Offset  = 0
	unsigned char barobuff = 0;
	uint32_t preasure=0;
	//best sample f
	//setOversampleRate(6);
	// -> Set back To BAROMETER mode with OSR = 128
	MPL3115A2_setModeBarometer();
	// null offset
	Baro_send( OFF_H, &barobuff, 1 );
	
	while(Baro_get_preasure(tlak)==1);
 	while(Baro_get_preasure(tlak)==1);
 	while(Baro_get_preasure(tlak)==1);
 	while(Baro_get_preasure(tlak)==1);


	for (uint8_t a=0;a<5;a++)
	{
		while(Baro_get_preasure(tlak)==1);
				 //  baro_meas = (float) ((Senzor.baro[0] << 16) | (Senzor.baro[1]<<8) | (Senzor.baro[2]) );
				   
		//temp+= (float) ((short) ((tlak[0] << 8) | tlak[1])) + (float) (tlak[2] >> 4) * 0.0625;
		temp+=(float) (((tlak[0] << 16) | (tlak[1] << 8) | (tlak[2] & 0xC0)) >> 6) + (float) ((tlak[2] & 0x30) >> 4) * 0.25;
// 		 Altitude = (float) ((short) ((OUT_P_MSB << 8) | OUT_P_CSB)) + (float) (OUT_P_LSB >> 4) * 0.0625;
// 		 preasure =  (tlak[0] << 16) | (tlak[1]<<8)| (tlak[2]) ;
// 		 preasure>>=6;
// 		 tlak[2] &= 0b00110000; //Bits 5/4 represent the fractional component
// 		 tlak[2]>>=4;
// 		 float pressure_decimal = (float)(tlak[2]/4.0); //Turn it into fraction
		 
		 //temp+=(float) (float)(preasure + pressure_decimal);
		vTaskDelay(2/portTICK_RATE_MS);
	}
	
	temp=(float)temp/5;
	double sea_pressure = temp * pow( 1-(*init_altitude)*0.0000225577, 5.255877 );
	//sea_pressure=10230;
	// Send this value now back to the sensor:
	unsigned char buffer[2];
	buffer[0] = (unsigned int)(sea_pressure / 2)>>8;
	buffer[1] = (unsigned int)(sea_pressure / 2)&0xFF;
	Baro_send(BAR_IN_MSB , buffer , 2 );
	vTaskDelay(100/portTICK_RATE_MS);
		/*set alt*/
	MPL3115A2_setModeAltimeter();
	//MPL3115A2_enableEventFlags();
	//MPL3115A2_setModeActive();
		
	vTaskDelay(100/portTICK_RATE_MS);
	while(Baro_get_preasure(tlak)==1);
	while(Baro_get_preasure(tlak)==1);
	
	
	temp=0;
	for (uint8_t p=0;p<10;p++)
	{
		while(Baro_get_preasure(tlak)==1);
		temp+=  (float) ((short) ((tlak[0] << 8) | tlak[1])) + (float) (tlak[2] >> 4) * 0.0625;
	
	}
				  
	*init_altitude=temp/10;



}


// By default I set the sensor to altimeter mode. I inserted a 1ms pause
// between each call to allow logic capture if needed. This give a small
// gap between captures on the bus to make working with the data easier.
void MPL3115A2_init()
{	
	MPL3115A2_setModeStandby();
	vTaskDelay(1/portTICK_RATE_MS);
	MPL3115A2_setModeAltimeter();
	vTaskDelay(1/portTICK_RATE_MS);
	setOversampleRate(6);
	vTaskDelay(1/portTICK_RATE_MS);
	MPL3115A2_enableEventFlags();
	vTaskDelay(1/portTICK_RATE_MS);
	MPL3115A2_setModeActive();
	vTaskDelay(1/portTICK_RATE_MS);
}

// This method wait for a specified amount of time for one of the data
// ready flags to be set. You need to pass in the correct data ready
// mask for this to work. See page 22 of the datasheet.
int MPL3115A2_dataReady(const char mask)
{
	uint8_t temp;
	Baro_read(STATUS,&temp,1);
	
	if ((temp&0x80)!=0x80)
	{
		//toggleOneShot(); //Toggle the OST bit causing the sensor to immediately take another reading
		return 1;
	}
	
	else
	{
		return 0;	//succes
	}
}



void MPL3115A2_setModeAltimeter()
{
	MPL3115A2_setRegisterBit(CTRL_REG1, 0x80);    // Set ALT bit
}

void MPL3115A2_setModeBarometer()
{
	MPL3115A2_clearRegisterBit(CTRL_REG1, 0x80);  // Clear ALT bit
}

void MPL3115A2_setModeStandby()
{
	MPL3115A2_clearRegisterBit(CTRL_REG1, 0x01);  // Clear SBYB bit for Standby mode
}
void MPL3115A2_setModeActive()
{
	MPL3115A2_setRegisterBit(CTRL_REG1, 0x01);    // Set SBYB bit for Active mode
}

void MPL3115A2_setOversampleRate(char sampleRate)
{
	if(sampleRate > 7)
	sampleRate = 7;                 // OS cannot be larger than 0b.0111
	
	sampleRate <<= 3;                   // Align it for the CTRL_REG1 register
	
	char temp = 0;
	Baro_read(CTRL_REG1,&temp,1);     // Read current settings
	temp &= 0xC7;                       // Clear out old OS bits
	temp |= sampleRate;                 // Mask in new OS bits
	Baro_send(CTRL_REG1, &temp,1);
}

void MPL3115A2_enableEventFlags()
{	
	uint8_t temp=0;
	temp=0x07;
	Baro_send(PT_DATA_CFG,&temp,1); // Enable all three pressure and temp event flags
}



void MPL3115A2_clearRegisterBit(unsigned char regAddr, unsigned char bitMask)
{
	unsigned char temp =0;
	Baro_read(regAddr,&temp,1);   // Read the current register value
	temp &= ~bitMask;               // Clear the bit from the value
	Baro_send(regAddr, &temp,1);        // Write register value back
}

void MPL3115A2_setRegisterBit(unsigned char regAddr, unsigned char bitMask)
{
	unsigned char temp =0;
	Baro_read(regAddr,&temp,1);   // Read the current register value
	temp |= bitMask;                // Set the bit in the value
	Baro_send(regAddr, &temp,1);        // Write register value back
}

int pressureDataReady() { return MPL3115A2_dataReady(0x04); }
//! Blocks for about 1/2 a second while checking the data ready bit. Returns 0 on sucees.
int temperatureDataReady() { return MPL3115A2_dataReady(0x02); }
 
 
 