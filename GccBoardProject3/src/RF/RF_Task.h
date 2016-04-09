/*
 * RF_Task.h
 *
 * Created: 14.2.2014 11:41:47
 *  Author: JR
 */ 


#ifndef RF_TASK_H_
#define RF_TASK_H_

#include "main.h"


void RF_Task(void *pvParameters);
void Send_data_LR(uint8_t *data,uint8_t Length);
void Send_data_FSK(uint8_t *data,uint8_t Length);
void RX_done_LR(RF_Queue *Semtech,short *crc);
void RX_done_FSK(RF_Queue *Semtech,short *crc);
uint8_t Check_status(char Line);
void Start_RX_LR(void);
void Start_RX_FSK(void);
void Rf_mode(RF_Queue *Sem_in);
void Semtech_IRQ1(void);
void Semtech_IRQ0(void);


#endif /* RF_TASK_H_ */