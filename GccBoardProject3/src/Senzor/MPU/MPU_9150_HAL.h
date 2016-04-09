/*
 * MPU_9150_HAL.h
 *
 * Created: 02.12.2015 21:52:08
 *  Author: uzivatel
 */ 


#ifndef MPU_9150_HAL_H_
#define MPU_9150_HAL_H_


uint32_t MPU_9150_send(unsigned char Adress, unsigned char *Data, unsigned char Length);
uint32_t MPU_9150_read(unsigned char Adress, unsigned char *Data, short Length);



#endif /* MPU_9150_HAL_H_ */