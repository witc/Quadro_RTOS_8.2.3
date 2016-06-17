/*
 * MS5611.c
 *
 * Created: 12.06.2016 21:10:37
 *  Author: J
 */ 
#include <asf.h>
#include <math.h>
#include <arm_math.h>
#include "FreeRTOS.h"
#include "main.h"
#include "Senzor_Task.h"
#include "FreeRTOS_V_8_Header.h"
#include "MS5611.h"
#include "MPL3115A2.h"


bool ms5611Detect(baro_t *baro)
{
	bool ack = false;
	uint8_t sig;
	int i;

	vTaskDelay(10/portTICK_RATE_MS); // No idea how long the chip takes to power-up, but let's make it 10ms
// 
// 	ack = i2cRead(MS5611_ADDR, CMD_PROM_RD, 1, &sig);
// 	if (!ack)
// 	return false;

	ms5611_reset();
	// read all coefficients from prom - calib constant
	for (i = 0; i < PROM_NB; i++)
	ms5611_c[i] = ms5611_prom(i);
	// check crc, bail out if wrong - we are probably talking to BMP085 w/o XCLR line!
	if (ms5611_crc(ms5611_c) != 0)
	return false;

	// TODO prom + CRC
	baro->ut_delay = 10000;
	baro->up_delay = 10000;
	baro->start_ut = ms5611_start_ut;
	baro->get_ut = ms5611_get_ut;
	baro->start_up = ms5611_start_up;
	baro->get_up = ms5611_get_up;
	baro->calculate = ms5611_calculate;

	return true;
}

static void ms5611_reset(void)
{	
	uint8_t temp=0;
	temp=CMD_RESET;
	Baro_send(MS5611_ADDR,&temp,1);
	vTaskDelay(3/portTICK_RATE_MS);
}

static uint16_t ms5611_prom(int8_t coef_num)
{
	uint8_t rxbuf[2] = { 0, 0 };
	uint8_t temp=0;
	temp=CMD_PROM_RD + coef_num * 2;
		
	Baro_send(MS5611_ADDR,&temp,1);
	Baro_read(MS5611_ADDR,rxbuf,2);

	return rxbuf[0] << 8 | rxbuf[1];
}

int8_t ms5611_crc(uint16_t *prom)
{
	int32_t i, j;
	uint32_t res = 0;
	uint8_t crc = prom[7] & 0xF;
	prom[7] &= 0xFF00;

	bool blankEeprom = true;

	for (i = 0; i < 16; i++) {
		if (prom[i >> 1]) {
			blankEeprom = false;
		}
		if (i & 1)
		res ^= ((prom[i >> 1]) & 0x00FF);
		else
		res ^= (prom[i >> 1] >> 8);
		for (j = 8; j > 0; j--) {
			if (res & 0x8000)
			res ^= 0x1800;
			res <<= 1;
		}
	}
	prom[7] |= crc;
	if (!blankEeprom && crc == ((res >> 12) & 0xF))
	return 0;

	return -1;
}

static uint32_t ms5611_read_adc(void)
{
	uint8_t rxbuf[3];
	Baro_read(MS5611_ADDR, rxbuf,3); // read ADC
	return (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
}

 void ms5611_start_ut(void)
{	
	uint8_t temp=0;
	/*cmd to	conversion*/
	temp=CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr;
	Baro_send(MS5611_ADDR,&temp,1);
	/*ted cekam na ACK az se dokonci mereni*/
	//i2cWrite(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr, 1); // D2 (temperature) conversion start!
}
uint32_t  ms5611_get_ut(void)
{	
	uint8_t temp=0;
	/*read ACK - sending read CMD - data are ready */
	temp=CMD_ADC_READ;
	Baro_send(MS5611_ADDR,&temp,1);
	
	ms5611_ut = ms5611_read_adc();
	return ms5611_ut;
	
}

 void ms5611_start_up(void)
{	
	uint8_t temp=0;
	temp= CMD_ADC_CONV + CMD_ADC_D1 + ms5611_osr;
	Baro_send(MS5611_ADDR,&temp,1);
	/*ted cekam na ACK az se dokonci mereni*/
	
}

uint32_t  ms5611_get_up(void)
{
	uint8_t temp=0;
	/*read ACK - sending read CMD - data are ready */
	temp=CMD_ADC_READ;
	
	ms5611_up = ms5611_read_adc();
	return ms5611_up;
}

 void ms5611_calculate(int32_t *pressure, int32_t *temperature)
{
	uint32_t press;
	int64_t temp;
	int64_t delt;
	int64_t dT = (int64_t)ms5611_ut - ((uint64_t)ms5611_c[5] * 256);
	int64_t off = ((int64_t)ms5611_c[2] << 16) + (((int64_t)ms5611_c[4] * dT) >> 7);
	int64_t sens = ((int64_t)ms5611_c[1] << 15) + (((int64_t)ms5611_c[3] * dT) >> 8);
	temp = 2000 + ((dT * (int64_t)ms5611_c[6]) >> 23);

	if (temp < 2000) { // temperature lower than 20degC
		delt = temp - 2000;
		delt = 5 * delt * delt;
		off -= delt >> 1;
		sens -= delt >> 2;
		if (temp < -1500) { // temperature lower than -15degC
			delt = temp + 1500;
			delt = delt * delt;
			off -= 7 * delt;
			sens -= (11 * delt) >> 1;
		}
		temp -= ((dT * dT) >> 31);
	}
	press = ((((int64_t)ms5611_up * sens) >> 21) - off) >> 15;


	if (pressure)
	*pressure = press;
	if (temperature)
	*temperature = temp;
}

