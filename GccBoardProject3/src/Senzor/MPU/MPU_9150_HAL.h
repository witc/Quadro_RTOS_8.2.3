/*
 * MPU_9150_HAL.h
 *
 * Created: 02.12.2015 21:52:08
 *  Author: uzivatel
 */ 


#ifndef MPU_9150_HAL_H_
#define MPU_9150_HAL_H_


uint32_t MPU_9150_send(unsigned char Adress, unsigned char Length, unsigned char *Data);
uint32_t MPU_9150_read(unsigned char Adress, short Length, unsigned char *Data);
uint32_t MPU_9150_send_mag(unsigned char Adress, unsigned char Length, unsigned char *Data);
uint32_t MPU_9150_read_mag(unsigned char Adress, unsigned char Length, unsigned char *Data);




#endif /* MPU_9150_HAL_H_ */