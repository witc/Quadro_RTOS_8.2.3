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
#include "GPS_Task.h"
#include "O_Flow_Task.h"

/* trace */
#include "trcUser.h"
#include "trcConfig.h"
#include "trcHardwarePort.h"

xTaskHandle		Sx1276_id;
xTaskHandle		Senzor_id;
xTaskHandle		Motor_id;
xTaskHandle		GPS_id;
xTaskHandle		O_FLOW_id;


volatile	xQueueHandle		Queue_RF_Task;
volatile	xQueueHandle		Queue_Senzor_Task;
volatile	xQueueHandle		Queue_Motor_Task;
volatile	xQueueHandle		Queue_GPS;
volatile	xQueueHandle		Queue_O_FLOW;


void System_TimerCallback(xTimerHandle pxTimer);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName);

volatile xTimerHandle System_Timer;
volatile uint32_t status_reset=0;

int main (void)
{
	
	sysclk_init();
	/*piin init*/
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
//	configure_console();
		
// 	 char buffer [10];
// 	 // 	 /* Clear the interrupt. */
// 	 status_reset=rstc_get_status(RSTC);
// 	 itoa (status_reset,buffer,10);
// 	 buffer[4]='\n';
// 	 usart_write_line((Usart*)UART1,buffer);
// 	 
// 	 status_reset&=0x700;
// 	 switch (status_reset)
// 	 {
// 		 case 0:
// 			 usart_write_line((Usart*)UART1,"General Reset...\n");
// 			 break;
// 			 
// 		case 0x100:
// 			 usart_write_line((Usart*)UART1,"Backup Reset...\n");
// 			 break;
// 		
// 		case 0x200:
// 			usart_write_line((Usart*)UART1,"Watch dog Reset...\n");
// 			break;
// 			
// 		case 0x300:
// 			usart_write_line((Usart*)UART1,"Soft Reset - pozadovany softwarem...\n");
// 			break;
// 		
// 		case 0x400:
// 			usart_write_line((Usart*)UART1,"User Reset - NRST pin detected low...\n");
// 			break;
// 	 }
	 
	Queue_RF_Task=xQueueCreate(5,sizeof(RF_Queue));
	Queue_Motor_Task=xQueueCreate(5,sizeof(Motor_Queue));
	Queue_Senzor_Task=xQueueCreate(10,sizeof(Senzor_Queue));
	Queue_GPS=xQueueCreate(2,sizeof(GPS_Queue));

  	vQueueAddToRegistry(Queue_RF_Task,"RF_Fronta");
  	vQueueAddToRegistry(Queue_Senzor_Task,"Senzor_Fronta");
  	vQueueAddToRegistry(Queue_Motor_Task,"Motor_Fronta");
  	vQueueAddToRegistry(Queue_GPS,"GPS_Fronta");
			
// 	vTraceSetQueueName(QueueTaskKeeperSPI_PDCA, "Keep_Q" );
// 	vTraceSetQueueName(QueueTaskCore, "Core_Q" );
	
	/* Timer blika ledkou - zarizeni zije*/
 	System_Timer=xTimerCreate("Timer_System",(500/portTICK_RATE_MS),pdTRUE,0,System_TimerCallback);
 	if(xTimerStart(System_Timer,0)!=pdPASS){}
		
	xTaskCreate(Senzor_Task,(const signed char * const) "Senzor",configMINIMAL_STACK_SIZE+400,NULL, 1,&Senzor_id);
	/*Create Semtech Task*/
	xTaskCreate(RF_Task,(const signed char * const) "Sx1276",configMINIMAL_STACK_SIZE+300,NULL, 1,&Sx1276_id);
	/* Create Motor task */
	xTaskCreate(Motor_Task,(const signed char * const) "Motor",configMINIMAL_STACK_SIZE+350,NULL, 1,&Motor_id);
	/*Create GPS Task*/
	xTaskCreate(GPS_Task,"GPS",configMINIMAL_STACK_SIZE+200,NULL, 1,&GPS_id);
	/*Create Optial Flow Task*/
//	xTaskCreate(O_Flow_Task,"O_Flow",configMINIMAL_STACK_SIZE+200,NULL, 1,&O_FLOW_id); 
	
	vTaskStartScheduler();
	
}



void System_TimerCallback(xTimerHandle pxTimer)
{	
		
	ioport_toggle_pin_level(LEDR);

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
//	usart_write_line((Usart*)UART1,"StackOwerflow...\n");
		   	
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
	//	usart_write_line((Usart*)UART1,"HardFault...\n");
	
		ioport_set_pin_level(LEDR,true);
		
 //}	
 __ASM volatile("BKPT #01");
 	while(1);
 }