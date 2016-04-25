/*
 * ADNS_3080_HAL.h
 *
 * Created: 16.04.2016 8:49:13
 *  Author: J
 */ 


#ifndef ADNS_3080-HAL_H_
#define ADNS_3080-HAL_H_

uint32_t ADNS_3080_send(unsigned char Adress, unsigned char *Data, unsigned char Length);
uint32_t ADNS_3080_read(unsigned char Adress, unsigned char *Data, short Length);




#endif /* ADNS_3080-HAL_H_ */