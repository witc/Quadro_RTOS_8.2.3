/*
 * MS5611.h
 *
 * Created: 12.06.2016 21:10:45
 *  Author: J
 */ 


#ifndef MS5611_H_
#define MS5611_H_

// MS5611, Standard address 0x77
#define MS5611_ADDR                 0x77

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8


typedef void (*baroOpFuncPtr)(void);                       // baro start operation
typedef void (*baroCalculateFuncPtr)(int32_t *pressure, int32_t *temperature); // baro calculation (filled params are pressure and temperature)

typedef struct baro_s {
	uint16_t ut_delay;
	uint16_t up_delay;
	baroOpFuncPtr start_ut;
	baroOpFuncPtr get_ut;
	baroOpFuncPtr start_up;
	baroOpFuncPtr get_up;
	baroCalculateFuncPtr calculate;
} baro_t;


static void ms5611_reset(void);
static uint16_t ms5611_prom(int8_t coef_num);
int8_t ms5611_crc(uint16_t *prom);
static uint32_t ms5611_read_adc(void);
 void ms5611_start_ut(void);
uint32_t ms5611_get_ut(void);
 void ms5611_start_up(void);
uint32_t  ms5611_get_up(void);
void ms5611_calculate(int32_t *pressure, int32_t *temperature);

uint32_t ms5611_ut;  // static result of temperature measurement
uint32_t ms5611_up;  // static result of pressure measurement
uint16_t ms5611_c[PROM_NB];  // on-chip ROM
static uint8_t ms5611_osr = CMD_ADC_4096;


bool ms5611Detect(baro_t *baro);

#endif /* MS5611_H_ */