/*
 * Comp_Hal.h
 *
 * Created: 16.7.2014 16:03:28
 *  Author: JR
 * Hardware Abstraction Layer - Hal
 
 */ 


#ifndef COMP_HAL_H_
#define COMP_HAL_H_

void Mag_read(unsigned char Adress, unsigned char *Data, unsigned char Length);
void Mag_send(unsigned char Adress, unsigned char *Data, unsigned char Length);


typedef struct{
	unsigned char bCRA_REG;
	unsigned char bCRB_REG;
	unsigned char bMR_REG;
	
} MY_MAG_CONF_STRUCT;


#define LIS_ADRESS				0x1E

#define CRA_REG_M_OFFSET		2
#define CRB_REG_M_OFFSET		5

#define ADRS_CRA_REG_M		0
#define ADRS_CRB_REG_M		1
#define ADRS_MR_REG_M		2
#define OUT_XHM				3
#define OUT_XLM				4
#define OUT_ZHM				5
#define OUT_ZLM				6
#define OUT_YHM				7
#define OUT_YLM				8
#define SR_REG_M			9

#endif /* COMP-HAL_H_ */