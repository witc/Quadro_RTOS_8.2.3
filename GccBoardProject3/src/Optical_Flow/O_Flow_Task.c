/*
 * O_Flow_Task.c
 *
 * Created: 15.04.2016 22:07:37
 *  Author: J
 */ 


#include <asf.h>
#include <math.h>
#include <arm_math.h>
#include "FreeRTOS_V_8_Header.h"
#include "ADNS_3080-HAL.h"

#include "main.h"
#include "O_Flow_Task.h"

extern	volatile	xQueueHandle		Queue_RF_Task;

extern volatile	xQueueHandle	Queue_O_FLOW;

uint8_t ADNS_INIT(void)
{	
	uint8_t temp=0;
	/*RESET */
	ioport_set_pin_level(ADNS_CS_PIN,false);
	vTaskDelay(1/portTICK_RATE_MS);
	ioport_set_pin_level(ADNS_CS_PIN,true);
	vTaskDelay(35/portTICK_RATE_MS);
	
	temp=0x44;
	ADNS_3080_send(0x20,&temp,1);
	temp=0x07;	
	ADNS_3080_send(0x23,&temp,1);	
	temp=0x88;	
	ADNS_3080_send(0x24,&temp,1);
	vTaskDelay(55/portTICK_RATE_MS);
	temp=0x18;	
	ADNS_3080_send(0x14,&temp,1);
	
	
 	
 	ADNS_3080_read(ADNS3080_PRODUCT_ID,&temp,1);
 	if (temp==0x17)	/* ID OK */;
 	else return -1;
	
		
}

/***********************************************/
void O_Flow_Task(void *pvParameters)
{
	int8_t motionDX=0;
	int8_t motionDY=0;
	uint8_t motion=0;
	RF_Queue Semtech;
	int16_t posli[4];
	// Read sensor
	uint8_t buf[4];
	uint8_t surfaceQuality=0;
	static int16_t x, y;
	
	/* init ADNS*/
	ADNS_INIT();
	
	for (;;)
	{
		
		
		 ADNS_3080_read(ADNS3080_MOTION_BURST, buf, 4);
 		 motion = buf[0];
		 motionDX = buf[1];
		 motionDY = buf[2];
		 surfaceQuality = buf[3];
// 		 //Serial.print(motion & 0x01); // Resolution
// 
// 		 if (motion & 0x10)
// 		 { // Check if we've had an overflow
// 			//Serial.println(F("ADNS-3080 overflow\n"));
// 		 }
// 		 else if (motion & 0x80)
// 		 {
// 			 motionDX = buf[1];
// 			 motionDY = buf[2];
// 			surfaceQuality = buf[3];

			 x += buf[0];
			 y += buf[1];
			
 	//	}	 
// 		ADNS_3080_read(REG_MOTION,&motion,1); // Freezes DX and DY until they are read or MOTION is read again.
// 		
// 		ADNS_3080_read(REG_DELTA_X,&motionDX,1); // Freezes DX and DY until they are read or MOTION is read again.
// 		ADNS_3080_read(REG_DELTA_Y,&motionDY,1); // Freezes DX and DY until they are read or MOTION is read again.
		
		/* posli do matlabu */
		posli[0]=(int16_t)x;//(P_PITCH*p_p);//+pid->I_p+pid->D_p
		posli[1]=(int16_t)y;//;(I_PITCH*integral_p);
		posli[2]=(int16_t)motion;//;(D_PITCH*derivative_p);
		posli[3]=(int16_t)0;//;position->pitch;
		
		Semtech.Buffer[0]=(uint8_t)posli[0];	//LOW
		Semtech.Buffer[1]=(uint8_t)(posli[0]>>8);		//HIGH
		Semtech.Buffer[2]=(uint8_t) posli[1];
		Semtech.Buffer[3]=(uint8_t) (posli[1]>>8);
		Semtech.Buffer[4]=(uint8_t) posli[2];
		Semtech.Buffer[5]=(uint8_t)( posli[2]>>8);
		Semtech.Buffer[6]=(uint8_t) posli[3];
		Semtech.Buffer[7]=(uint8_t)( posli[3]>>8);
		
		Semtech.Stat.Data_State=RFLR_STATE_TX_INIT;
		Semtech.Stat.Cmd=STAY_IN_STATE;
		/* Send data to Matlab */

// 		   	for (uint8_t i=0;i<2;i++)
// 		   	{
// 		   		usart_putchar((Usart*)UART1,Semtech.Buffer[0+i]);
// 		    	}
 		if(xQueueSend(Queue_RF_Task,&Semtech,portMAX_DELAY)==pdPASS)	//pdPASS=1-
 		{
 		
 		}
		 vTaskDelay(5/portTICK_RATE_MS);
	}
}