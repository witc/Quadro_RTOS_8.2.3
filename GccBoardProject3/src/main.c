/**
 * \file
 *
 * \brief Empty user application template
 *
 */

#include <asf.h>
#include "FreeRTOS_V_8_Header.h"
#include "stdlib.h"

#include "Senzor_Task.h"
#include "RF_Task.h"
#include "Periph_init.h"
#include "Motor.h"

/* trace */
#include "trcUser.h"
#include "trcConfig.h"
#include "trcHardwarePort.h"

xTaskHandle		Sx1276_id;
xTaskHandle		Senzor_id;
xTaskHandle		Motor_id;


volatile	xQueueHandle		Queue_RF_Task;
volatile	xQueueHandle		Queue_Senzor_Task;
volatile	xQueueHandle		Queue_Motor_Task;


void System_TimerCallback(xTimerHandle pxTimer);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName);

volatile xTimerHandle System_Timer;
volatile uint32_t status_reset=0;

int main (void)
{
	
	sysclk_init();
	board_init();
	/* vsechny pouzite sbernice init */
	bus_init();
	/*  */
	wdt_disable(WDT);
		 
	cpu_irq_enable();
	/* Initialize trace library before using any FreeRTOS APIs if enabled */
 	vTraceInitTraceData();
 	/* Start tracing */
	uiTraceStart();
	
	/*for debug only */
	ioport_set_pin_level(PERIODE_PIN,true);
	configure_console();
		
	 char buffer [10];
	 // 	 /* Clear the interrupt. */
	 status_reset=rstc_get_status(RSTC);
	 itoa (status_reset,buffer,10);
	 buffer[4]='\n';
	 usart_write_line((Usart*)UART1,buffer);
	 
	 status_reset&=0x700;
	 switch (status_reset)
	 {
		 case 0:
			 usart_write_line((Usart*)UART1,"General Reset...\n");
			 break;
			 
		case 0x100:
			 usart_write_line((Usart*)UART1,"Backup Reset...\n");
			 break;
		
		case 0x200:
			usart_write_line((Usart*)UART1,"Watch dog Reset...\n");
			break;
			
		case 0x300:
			usart_write_line((Usart*)UART1,"Soft Reset - pozadovany softwarem...\n");
			break;
		
		case 0x400:
			usart_write_line((Usart*)UART1,"User Reset - NRST pin detected low...\n");
			break;
	 }
	 
	Queue_RF_Task=xQueueCreate(5,sizeof(RF_Queue));
	Queue_Motor_Task=xQueueCreate(5,sizeof(Motor_Queue));
	Queue_Senzor_Task=xQueueCreate(10,sizeof(MPU9150_Queue));	

  	vQueueAddToRegistry(Queue_RF_Task,"RF_Fronta");
  	vQueueAddToRegistry(Queue_Senzor_Task,"Senzor_Fronta");
  	vQueueAddToRegistry(Queue_Motor_Task,"Motor_Fronta");
			
// 	vTraceSetQueueName(QueueTaskKeeperSPI_PDCA, "Keep_Q" );
// 	vTraceSetQueueName(QueueTaskCore, "Core_Q" );
	
	/* Timer blika ledkou - zarizeni zije*/
 	System_Timer=xTimerCreate("Timer_System",(1000/portTICK_RATE_MS),pdTRUE,0,System_TimerCallback);
 	if(xTimerStart(System_Timer,0)!=pdPASS){}
		
	xTaskCreate(Senzor_Task,(const signed char * const) "Senzor",configMINIMAL_STACK_SIZE+400,NULL, 1,&Senzor_id);
	/*Create Semtech Task*/
	xTaskCreate(RF_Task,(const signed char * const) "Sx1276",configMINIMAL_STACK_SIZE+400,NULL, 1,&Sx1276_id);
	/* Create Motor task */
	xTaskCreate(Motor_Task,(const signed char * const) "Motor",configMINIMAL_STACK_SIZE+400,NULL, 1,&Motor_id);
	
	
	vTaskStartScheduler();
	
}



void System_TimerCallback(xTimerHandle pxTimer)
{	
	Motor_Queue Position;
	static uint8_t RFBuffer[8];
	volatile  uint16_t ch0_result;
	volatile uint16_t ch1_result;
	volatile uint16_t ch2_result;
	volatile uint16_t ch3_result;
	static uint8_t tempa=0;
	//tempa++;
	if (tempa==0)
	{
		ch0_result=2700;
		ch1_result=2700;
		ch2_result=2700;
		ch3_result=2700;
		tempa=1;
	}else
	{
		ch0_result=23300;
		ch1_result=2300;
		ch2_result=2300;
		ch3_result=2300;
		tempa=0;
	}
	
	
	ioport_toggle_pin_level(LEDR);
	
	RFBuffer[0]=(uint8_t)ch0_result;
	RFBuffer[1]=(uint8_t)(ch0_result>>8);
	
	RFBuffer[2]=(uint8_t)ch1_result;
	RFBuffer[3]=(uint8_t)(ch1_result>>8);
	
	RFBuffer[4]=(uint8_t)ch2_result;
	RFBuffer[5]=(uint8_t)(ch2_result>>8);
	
	RFBuffer[6]=(uint8_t)ch3_result;
	RFBuffer[7]=(uint8_t)(ch3_result>>8);
	
	Position.TX_CH_xx[0]=RFBuffer[0];
	Position.TX_CH_xx[0]|=(RFBuffer[1]<<8);
	
	Position.TX_CH_xx[1]=RFBuffer[2];
	Position.TX_CH_xx[1]|=(RFBuffer[3]<<8);
	
	Position.TX_CH_xx[2]=RFBuffer[4];
	Position.TX_CH_xx[2]|=(RFBuffer[5]<<8);
	
	Position.TX_CH_xx[3]=RFBuffer[6];
	Position.TX_CH_xx[3]|=(RFBuffer[7]<<8);
	
	/* Send new TX data to motor task */
// 	Position.type_of_data=FROM_TX;
// 	if(xQueueSend(Queue_Motor_Task,&Position,portMAX_DELAY))	//pdPASS=1-
// 	{
// 		
// 	}
	//usart_write_line((Usart*)UART1,"SystemTimer_tick...\n");
	
}



//	if No task to execute	*/
void vApplicationIdleHook(void)
{	
	
	while(1){

		
	}
}
	
	
/* FreeRTOS stack overflow hook */
void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
    
	(void) pxTask;
    (void) pcTaskName;
   //  static char pole[100];
// 		
//  	for (short i=0;i<strlen(pcTaskName);i++)
//  	{
//  		pole[i]=pcTaskName[i];
//  	}
//	sprintf(pole );
	ioport_set_pin_level(LEDR,true);
	usart_write_line((Usart*)UART1,"StackOwerflow...\n");
		   	
	__ASM volatile("BKPT #01");	
    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    	
    for (;; ) {}
}		


 void HardFault_Handler(void)
 {	
//  	static char zprava1[80];
//  	static char zprava2[80];
//  		
//  	sprintf(zprava1, "SCB->HFSR = 0x%08x\n", SCB->HFSR);
//  	//nastal Hardfault
//  	if ((SCB->HFSR & (1 << 30)) != 0) {
//  	
//  	sprintf(zprava2, "SCB->CFSR = 0x%08x\n", SCB->CFSR );
		usart_write_line((Usart*)UART1,"HardFault...\n");
	
		ioport_set_pin_level(LEDR,true);
		
 //}	
 __ASM volatile("BKPT #01");
 	while(1);
 }