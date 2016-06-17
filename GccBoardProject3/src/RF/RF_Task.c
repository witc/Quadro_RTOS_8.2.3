/*
 * RF_Task.c
 *
 * Created: 14.2.2014 16:41:00
 *  Author: JR
 */ 


#include <asf.h>
#include <stdlib.h>
#include "RF_Task.h"
#include "main.h"
#include "sx1276-Hal.h"
#include "sx1276.h"
#include "sx1276-Fchp.h"
#include "sx1276-LoRa.h"
#include "FreeRTOS_V_8_Header.h"


/* Define a type of modulation*/
#define LORA	1
#define FSK		0

#define RSSI_OFFSET                                 -137.0
#define NOISE_ABSOLUTE_ZERO                         -174.0
#define NOISE_FIGURE                                6.0

extern	volatile	xQueueHandle		Queue_RF_Task;
extern volatile		xSemaphoreHandle	Lights_RF_Busy;
extern xTaskHandle		Sx1276_id;
extern volatile	xQueueHandle		Queue_Motor_Task;


uint8_t pole[]={1,2,3,4,5,6};

// Default settings
extern tLoRaSettings LoRaSettings;
extern tSX1276LR SX1276LR;

/*NIRQ0 - From RF Semtech - RX Done*/
void Semtech_IRQ0(void)
{
	RF_Queue	Semtech;
	
	signed portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken=pdTRUE;	//pøerušení se dokonèí celé= pdFalse
	//pio_disable_interrupt(PIOA, PIO_PA18);
	
	Semtech.Stat.Cmd=STAY_IN_STATE;
	Semtech.Stat.Data_State=Check_status(0);
	//xTaskResumeFromISR(Sx1276_id);
	//while(xQueueSendToBackFromISR(Queue_RF_Task,&Semtech,&xHigherPriorityTaskWoken)!=pdTRUE);	//lepší používat Back..
	 //pio_enable_interrupt(PIOA, PIO_PA18);
	xQueueSendToBackFromISR(Queue_RF_Task,&Semtech,&xHigherPriorityTaskWoken);
	
	
	if (xHigherPriorityTaskWoken==pdTRUE) portYIELD();
	
	
}

/*NIRQ1 - From RF Semtech - Timeout*/
void Semtech_IRQ1(void)
{
	RF_Queue	Semtech;
	signed portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken=pdTRUE;	//pøerušení se dokonèí celé= pdFalse
	
	//pio_disable_interrupt(PIOA, PIO_PA17);
	
	Semtech.Stat.Cmd=STAY_IN_STATE;
	Semtech.Stat.Data_State=Check_status(1);
	//pio_enable_interrupt(PIOA, PIO_PA17);
	
	xQueueSendToBackFromISR(Queue_RF_Task,&Semtech,&xHigherPriorityTaskWoken);
	
	
	if (xHigherPriorityTaskWoken==pdTRUE) portYIELD();
	
	
}


/****************************************************************************/
void RX_done_LR(RF_Queue *Semtech,short *crc)
{	
	static short RxPacketSize = 0;
	static uint8_t RFBuffer[RF_BUFFER_SIZE];
	Motor_Queue Position;

	
	SX1276LoRaSetOpMode(RFLR_OPMODE_STANDBY);
	
	//SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE  );
	SX1276Read( REG_LR_IRQFLAGS, &SX1276LR.RegIrqFlags );
	
	if( ( SX1276LR.RegIrqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
	{
		// Clear Irq
		 SX1276Write( REG_LR_IRQFLAGS,SX1276LR.RegIrqFlags  );
		//gpio_set_pin_high(LED0);
		//Semtech->State = RFLR_STATE_RX_INIT;
		*crc=CRC_FALSE;
		//Semtech->Rssi=SX1276LoRaReadRssi();
				
	}
	else if ((SX1276LR.RegIrqFlags & RFLR_IRQFLAGS_RXDONE ) == RFLR_IRQFLAGS_RXDONE)
	{
		*crc=CRC_OK;
		SX1276Write( REG_LR_IRQFLAGS,SX1276LR.RegIrqFlags ); //RFLR_IRQFLAGS_RXDONE_MASK
		//ioport_toggle_pin(LED_OK);
		ioport_toggle_pin_level(LEDW);
		
		//RxPacketRssiValue=SX1276LoRaReadRssi();
		
		if( LoRaSettings.RxSingleOn == true ) // Rx single mode
		{
			
			SX1276LR.RegFifoAddrPtr =SX1276LR.RegFifoRxBaseAddr;;
			SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR.RegFifoAddrPtr );

			if( LoRaSettings.ImplicitHeaderOn == true )
			{
				RxPacketSize = SX1276LR.RegPayloadLength;
				SX1276ReadFifo( RFBuffer,RxPacketSize); //SX1276LR->RegPayloadLength
			}
			else
			{
				SX1276Read( REG_LR_NBRXBYTES, &SX1276LR.RegNbRxBytes );	//Nuber of recieved bytes
				RxPacketSize = SX1276LR.RegNbRxBytes;
				SX1276ReadFifo( RFBuffer, RxPacketSize); //
			}
		}
		else // Rx continuous mode
		{
			SX1276Read( REG_LR_FIFORXCURRENTADDR, &SX1276LR.RegFifoRxCurrentAddr );

			if( LoRaSettings.ImplicitHeaderOn == true )
			{
				RxPacketSize = SX1276LR.RegPayloadLength;
				SX1276LR.RegFifoAddrPtr = SX1276LR.RegFifoRxCurrentAddr - SX1276LR.RegPayloadLength;
				SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR.RegFifoAddrPtr );
				SX1276ReadFifo( RFBuffer, SX1276LR.RegPayloadLength );
			}
			else
			{
				
				SX1276Read( REG_LR_NBRXBYTES, &SX1276LR.RegNbRxBytes );
				RxPacketSize = SX1276LR.RegNbRxBytes;
				SX1276LR.RegFifoAddrPtr = SX1276LR.RegFifoRxCurrentAddr - SX1276LR.RegNbRxBytes;
				SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR.RegFifoAddrPtr );
				SX1276ReadFifo( RFBuffer, SX1276LR.RegNbRxBytes );
			}
		}
		
			
		
		Position.TX_CH_xx[0]=RFBuffer[0];
		Position.TX_CH_xx[0]|=(RFBuffer[1]<<8);
		
		Position.TX_CH_xx[1]=RFBuffer[2];
		Position.TX_CH_xx[1]|=(RFBuffer[3]<<8);
		
		Position.TX_CH_xx[2]=RFBuffer[4];
		Position.TX_CH_xx[2]|=(RFBuffer[5]<<8);
		
		Position.TX_CH_xx[3]=RFBuffer[6];
		Position.TX_CH_xx[3]|=(RFBuffer[7]<<8);
		
		/* Send new TX data to motor task */
		Position.type_of_data=FROM_TX;
		if(xQueueSend(Queue_Motor_Task,&Position,portMAX_DELAY))	//pdPASS=1-
		{
			
		}
 	
	//Clear all irqs in SEMTECH
	}else
	{
		SX1276Write( REG_LR_IRQFLAGS, 0xFF );
		//Semtech->State = RFLR_STATE_RX_INIT;
	//	RxPacketSize = SX1276LR.RegPayloadLength;
	//	SX1276ReadFifo( RFBuffer,RxPacketSize); //SX1276LR->RegPayloadLength 
			
	}
		
	
//	NVIC_EnableIRQ(GPS_IRQ);
	
	
}
/************************************************************************/

void Start_RX_LR(void)
{	
	SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
		 SX1276Write( REG_LR_IRQFLAGS,0xFF  );
	
	SX1276LR.RegIrqFlagsMask =
	//RFLR_IRQFLAGS_RXTIMEOUT |
	//RFLR_IRQFLAGS_RXDONE|
	//RFLR_IRQFLAGS_PAYLOADCRCERROR |
	RFLR_IRQFLAGS_VALIDHEADER |
	RFLR_IRQFLAGS_TXDONE |
	RFLR_IRQFLAGS_CADDONE |
	RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
	RFLR_IRQFLAGS_CADDETECTED;
	
	SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR.RegIrqFlagsMask );

	SX1276LR.RegHopPeriod = 0;	//Datasheet says 0 ... nebo stara verze 255?
	SX1276Write( REG_LR_HOPPERIOD, SX1276LR.RegHopPeriod );
	// RxDone                    RxTimeout                   FhssChangeChannel           CadDone
	SX1276LR.RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_00;// |
	RFLR_DIOMAPPING1_DIO1_00;// | 	//RXTimeout
	// RFLR_DIOMAPPING1_DIO2_00 |
	// RFLR_DIOMAPPING1_DIO3_10; //CRC error
	// CadDetected               ModeReady
	SX1276LR.RegDioMapping2 =0;// RFLR_DIOMAPPING2_DIO4_00 | RFLR_DIOMAPPING2_DIO5_00;
	SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR.RegDioMapping1, 2 );

	// 	 // see errata note
	//SX1276Write( 0x2F, 0x14 );
	//SX1276LoRaSetPayloadLength( LoRaSettings.PayloadLength,&SX1276LR );	//
	
	if( LoRaSettings.RxSingleOn == true ) // Rx single mode
	{
		SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER_SINGLE );
	}
	else // Rx continuous mode
	{
		SX1276LR.RegFifoAddrPtr = SX1276LR.RegFifoRxBaseAddr;
		SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR.RegFifoAddrPtr );
		
		SX1276LoRaSetOpMode( RFLR_OPMODE_RECEIVER );
	}
	
	
		
}

/************************************************************************/
void Send_data_LR(uint8_t *data,uint8_t Length)
{	
	
	//uint8_t Temp=0;
//	uint16_t Timeout=2600;	// u kzadeho oboju musi byt jinak ?!?!?
				
	// see errata note
	//SX1276LoRaSetOpMode( RFLR_OPMODE_SLEEP );
	SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY );
	
// 	SX1276Read(REG_LR_IRQFLAGSMASK,&Temp);
 //	SX1276Write( REG_LR_IRQFLAGS, 0xFF  );
	
	SX1276LR.RegIrqFlagsMask =
	RFLR_IRQFLAGS_RXTIMEOUT |
	RFLR_IRQFLAGS_RXDONE |
	RFLR_IRQFLAGS_PAYLOADCRCERROR |
	RFLR_IRQFLAGS_VALIDHEADER |
	//RFLR_IRQFLAGS_TXDONE |
	RFLR_IRQFLAGS_CADDONE |
	RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL
	|RFLR_IRQFLAGS_CADDETECTED
	;
	
	SX1276LR.RegHopPeriod = 0;
	
	SX1276Write(REG_LR_HOPPERIOD, SX1276LR.RegHopPeriod );	//0x1C
	SX1276Write( REG_LR_IRQFLAGSMASK, SX1276LR.RegIrqFlagsMask );

	// Initializes the payload size
	SX1276LR.RegPayloadLength = Length;
	SX1276Write(REG_LR_PAYLOADLENGTH, SX1276LR.RegPayloadLength );	//0x17
	
	SX1276LR.RegFifoTxBaseAddr = 0x00; // Full buffer used for Tx
	SX1276Write( REG_LR_FIFOTXBASEADDR, SX1276LR.RegFifoTxBaseAddr );

	SX1276LR.RegFifoAddrPtr = SX1276LR.RegFifoTxBaseAddr;
	SX1276Write( REG_LR_FIFOADDRPTR, SX1276LR.RegFifoAddrPtr );
	
	// Write payload buffer to LORA modem
	SX1276WriteFifo(data,Length);
	
	///TX done						//CAD DONE							//Detected CAD
	SX1276LR.RegDioMapping1 = RFLR_DIOMAPPING1_DIO0_01;// | RFLR_DIOMAPPING1_DIO0_10 |RFLR_DIOMAPPING1_DIO1_10 ;//| RFLR_DIOMAPPING1_DIO1_00 | RFLR_DIOMAPPING1_DIO2_00 | RFLR_DIOMAPPING1_DIO3_01;
	// PllLock              Mode Ready
	SX1276LR.RegDioMapping2 = 0;//RFLR_DIOMAPPING2_DIO4_01 | RFLR_DIOMAPPING2_DIO5_00;
	SX1276WriteBuffer( REG_LR_DIOMAPPING1, &SX1276LR.RegDioMapping1, 2 );
	
	/* aby matlab stihal vykreslovat*/
 	vTaskDelay(2/portTICK_RATE_MS);
 	 
	SX1276LoRaSetOpMode( RFLR_OPMODE_TRANSMITTER );
	

}



uint8_t Check_status(char Line)
{
	//uint8_t Temp;
	//RF_Queue Semtech;
		
		
		if (Line==0)
		{
			//SX1276Write( 0x12, RFLR_IRQFLAGS_RXDONE_MASK);
#if (RX_NEW_CMD==1)
			return	RFLR_STATE_RX_DONE;
#elif (TX_TO_MATLAB==1)			

			return	RFLR_STATE_TX_DONE;
#else
# error "TX or RX?"
#endif
			 
		}else if(Line==1)
		{
			// SX1276Write( 0x12, RFLR_IRQFLAGS_RXTIMEOUT_MASK);//
			 return	RFLR_STATE_RX_TIMEOUT;
			 
		}else if(Line==2)
		{
			SX1276Write( 0x12,RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK);
			return RFLR_STATE_CRC_ERROR;
		}else
		{
		 	//SX1276Write( 0x12, 0);
		 	return 0x66;
		}
	
}

void Rf_mode(RF_Queue *Sem_in)
{	
	RF_Queue Semtech;
	short Valid_packet=0;
	static uint8_t TX_READY=1;

	switch (Sem_in->Stat.Data_State)
	{
		
		case RFLR_STATE_IDLE:
			

			break;
		
		case RFLR_STATE_ERROR:
			Semtech.Stat.Data_State=RFLR_STATE_RX_INIT;
			Semtech.Stat.Cmd=STAY_IN_STATE;			
			xQueueSend(Queue_RF_Task,&Semtech,3000);
		
			break;
		
		case RFLR_STATE_RX_INIT:
			#ifdef LORA
				Start_RX_LR();
				
			#else
				Start_RX_FSK();
			#endif
			
			break;
		
		case RFLR_STATE_RX_RUNNING:
		
			break;
		
		case RFLR_STATE_RX_DONE:
			
			#ifdef LORA
					RX_done_LR(&Semtech,&Valid_packet);
			#else
					RX_done_FSK(&Semtech,&Valid_packet);
			#endif
			
			Semtech.Stat.Data_State=RFLR_STATE_RX_INIT;
			Semtech.Stat.Cmd=STAY_IN_STATE;
			if(xQueueSend(Queue_RF_Task,&Semtech,portMAX_DELAY)!=pdPASS)
			{
		
			}
		
			
		
			break;
		
		case RFLR_STATE_RX_TIMEOUT:
			
			SX1276LoRaSetOpMode(RFLR_OPMODE_STANDBY);
			SX1276Write( REG_LR_IRQFLAGS, 0xFF );
			Semtech.Stat.Cmd=STAY_IN_STATE;
			Semtech.Stat.Data_State=RFLR_STATE_RX_INIT;
			ioport_toggle_pin_level(LEDW);
	
			if(xQueueSend(Queue_RF_Task,&Semtech,portMAX_DELAY)!=pdPASS)
			{
				
			}
			
			break;
		
		case RFLR_STATE_CRC_ERROR:	//nepoužito
		
			Semtech.Stat.Cmd=STAY_IN_STATE;
			Semtech.Stat.Data_State=RFLR_STATE_RX_INIT;
			if(xQueueSend(Queue_RF_Task,&Semtech,10000)!=pdPASS)
			{
				
			}
			break;
		
		case RFLR_STATE_TX_INIT:
			
			if (TX_READY==1)
			{
				TX_READY=0;
				#if (LORA==1)
				Send_data_LR(Sem_in->Buffer,LoRaSettings.PayloadLength);
// 				 	for (uint8_t i=0;i<2;i++)
// 				 	{
// 					 	usart_putchar((Usart*)UART1,Semtech.Buffer[6+i]);
// 				 	}
			
				#elif (FSK==1)
				Send_data_FSK(Sem_in->Buffer,6);
				
				#else
				#error "NO LORA or FSK Defined"
				
				#endif
				
// 				Semtech.Stat.Cmd=STAY_IN_STATE;
// 				Semtech.Stat.Data_State=RFLR_STATE_TX_RUNNING;
// 				if(xQueueSend(Queue_RF_Task,&Semtech,portMAX_DELAY)!=pdPASS)
// 				{
// 					
// 				}
			
			}
			
			break;
		
		case RFLR_STATE_TX_RUNNING:
		
			break;
		
		case RFLR_STATE_TX_DONE:
			
			TX_READY=1;
			ioport_toggle_pin_level(LEDW);
		
			SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE  );	
			// optimize the power consumption by switching off the transmitter as soon as the packet has been sent
			SX1276LoRaSetOpMode( RFLR_OPMODE_STANDBY);
			SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE  );
		//	TX_READY=1;
// 			Semtech.Stat.Cmd=STAY_IN_STATE;
// 			Semtech.Stat.Data_State=RFLR_STATE_TX_INIT;
// 			if(xQueueSend(Queue_RF_Task,&Semtech,10000)!=pdPASS)
// 			{
// 				
// 			}
			
			break;
			
		
		
		default:
			break;
	}
}

/**************************************************************************/
void RF_Task(void *pvParameters)
{
	RF_Queue Semtech;
	//MANAGER_TASK Manage_data;
	
#if (RX_NEW_CMD==1)
	Semtech.Stat.Data_State=RFLR_STATE_RX_INIT;
	Semtech.Stat.Cmd=STAY_IN_STATE;
	xQueueSend(Queue_RF_Task,&Semtech,portMAX_DELAY);
#elif (TX_TO_MATLAB==1)

#else
# error "TX or RX?"
#endif
 
	//taskENTER_CRITICAL();
	SX1276Init();
 	vTaskDelay(1/portTICK_RATE_MS);
	//taskEXIT_CRITICAL();
	
	/* Semtech int*/
	//  	pio_set_input(PIOA, PIO_PA17_IDX, PIO_DEFAULT);
	//  	pio_set_input(PIOA, PIO_PA18_IDX, PIO_DEFAULT);
	pio_handler_set(PIOA, ID_PIOA, PIO_PA18, PIO_IT_RISE_EDGE, Semtech_IRQ0);
	pio_handler_set(PIOA, ID_PIOA, PIO_PA17, PIO_IT_RISE_EDGE, Semtech_IRQ1);
	pio_handler_set_priority(PIOA, PIOA_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	pio_enable_interrupt(PIOA, PIO_PA18);
	pio_enable_interrupt(PIOA, PIO_PA17);
	
	NVIC_DisableIRQ((IRQn_Type)PIOA_IRQn);
	NVIC_ClearPendingIRQ((IRQn_Type) PIOA_IRQn);
	NVIC_EnableIRQ((IRQn_Type)PIOA_IRQn);
	NVIC_SetPriority((IRQn_Type)PIOA_IRQn,configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	
	for (;;)
	{	
	
  		if(xQueueReceive(Queue_RF_Task,&Semtech,portMAX_DELAY)==pdPASS)
		{
 						
  			if (Semtech.Stat.Cmd==CHANGE_STATE)
  			{
  				if (Semtech.Stat.Data_State==STATE_OFF)
  				{
  					SX1276Write( REG_LR_IRQFLAGS, 0xFF );
  					SX1276LoRaSetOpMode(RFLR_OPMODE_SLEEP);
  // 					Manage_data.Task=RF_i;
  // 					Manage_data.State_RDY=RDY_TO_SLEEP;
  // 					xQueueSend(Queue_Manage,&Manage_data,portMAX_DELAY);
  					//xSemaphoreGive(Lights_Distance);
  				}
  				else if(Semtech.Stat.Data_State==STATE_ON)
  				{
  					taskENTER_CRITICAL();
  					SX1276Init();
  					vTaskDelay(1/portTICK_RATE_MS);
  					taskEXIT_CRITICAL();
  				}
  				
  			}
  			else if(Semtech.Stat.Cmd==STAY_IN_STATE)
  			{
  				Rf_mode(&Semtech);
  			}
		
		
		//fronta prazdna po xx ticich
		}else
		{
// 			Semtech.Stat.Data_State=RFLR_STATE_RX_INIT;
// 			Semtech.Stat.Cmd=STAY_IN_STATE;
// 			if(xQueueSend(Queue_RF_Task,&Semtech,5000)!=pdPASS)
// 			{
// 				
// 			}
		}
		
		

	
	}
}

/*
 *Mutex:

 Is a key to a toilet. One person can have the key - occupy the toilet - at the time. When finished, the person gives (frees) the key to the next person in the queue.

 Officially: "Mutexes are typically used to serialise access to a section of re-entrant code that cannot be executed concurrently by more than one thread. A mutex object only allows one thread into a controlled section, forcing other threads which attempt to gain access to that section to wait until the first thread has exited from that section." Ref: Symbian Developer Library

 (A mutex is really a semaphore with value 1.)

 Semaphore:

 Is the number of free identical toilet keys. Example, say we have four toilets with identical locks and keys. The semaphore count - the count of keys - is set to 4 at beginning (all four toilets are free), then the count value is decremented as people are coming in. If all toilets are full, ie. there are no free keys left, the semaphore count is 0. Now, when eq. one person leaves the toilet, semaphore is increased to 1 (one free key), and given to the next person in the queue.

 Officially: "A semaphore restricts the number of simultaneous users of a shared resource up to a maximum number. Threads can request access to the resource (decrementing the semaphore), and can signal that they have finished using the resource (incrementing the semaphore)." Ref: Symbian Developer Library
 */