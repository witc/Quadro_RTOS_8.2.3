/*
 * GPS_Task.c
 *
 * Created: 27.1.2014 11:33:24
 *  Author: JR
 */ 
#include <asf.h>
#include <stdlib.h>
#include "FreeRTOS_V_8_Header.h"
#include "string.h"
#include "main.h"
#include "math.h"
#include "GPS_Task.h"
#include "GPS_Driver.h"


extern	volatile	xQueueHandle		Queue_GPS;
extern volatile	 xQueueHandle		    Queue_Senzor_Task;


void USART1_Handler(void)
{	
	static short counter=0;
	irqflags_t LocSREG;
	uint32_t ul_status;
	static GPS_Queue DataToGPS;
	signed portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken=pdTRUE;	//pøerušení se dokonèí celé= pdFalse
	// 
	//taskENTER_CRITICAL();
	usart_getchar(GPS_USART,&DataToGPS.Nmea_data[counter]);
	
	if ((DataToGPS.Nmea_data[counter]=='\n')&&(DataToGPS.Nmea_data[0]==0x24))
	{
		//usart_disable_interrupt(GPS_USART,US_IER_RXRDY);
		//usart_reset_status(CONF_UART);
		//usart_disable_rx(CONF_UART);
		DataToGPS.Nmea_data[counter++]=0;	// pro praci se strlen - konec musi byt roven 0, '\0' http://mcu.cz/plugins/smf/smf.php?topic=3299.18
		counter=0;
		DataToGPS.GL_flag_gps=SENTENCE_OK;
		
	
		//taskEXIT_CRITICAL();
		xQueueSendFromISR(Queue_GPS,&DataToGPS,&xHigherPriorityTaskWoken);
			
	}else
	{
		if ((counter<77)&&(DataToGPS.Nmea_data[0]==0x24)) counter++;//
		//taskEXIT_CRITICAL();
		
	}
	
	

}


/******************************************************************/


// void Dma_init(void)
// {	
// 	/* Get pointer to UART PDC register base */
// 	gPS_Usart = uart_get_pdc_base(CONF_UART);
// 
// 	/* Initialize PDC data packet for transfer */
// 	gPS_DMA_para.ul_addr = (uint32_t)GPS_DMA_Buffer;
// 	gPS_DMA_para.ul_size = 100;
// 
// 	/* Configure PDC for data receive */
// 	pdc_rx_init(gPS_Usart, &gPS_DMA_para, NULL);
// 
// 	/* Enable PDC transfers */
// 	pdc_enable_transfer(gPS_Usart, PERIPH_PTCR_RXTEN);
// 
// 	/* Enable UART IRQ */
// 	uart_enable_interrupt(CONF_UART, UART_IER_RXBUFF);
// 
// 	/* Enable UsART1 interrupt */
// 	NVIC_EnableIRQ(USART1_IRQn);
// 	
// // 	pdca_enable(PDCA);	//Enable clock for PDCA - DMA
// // 	pdca_channel_set_config(PDCA_RX_CHANNEL, &PDCA_RX_CONFIGS);
// // 	pdca_channel_set_callback(PDCA_RX_CHANNEL, PDCA_GPS_Done,PDCA_USART_LINE, 1, PDCA_IER_TRC);	//Transefer completet IRQ
// // 	//pdca_channel_write_load(PDCA_RX_CHANNEL, &uc_receive, 1);
// // 	
// // 	pdca_channel_enable(PDCA_RX_CHANNEL);
// 	
// 	
// }
/***************************************************************************/

 char GPS_Utils_CalcDisAndBear( GPS_POSITION_t* psGPS_PositionMaster, GPS_POSITION_t* psGPS_PositionSlave, GPS_COMP_DATA_t* psCOMP_Data )
 {
	 //---------------------------------------------------------------------
	 // WGS-84 ellipsoid params
	 double a = 6378137;
	 double b = 6356752.314245;
	 double f = 1/298.257223563;
	 //---------------------------------------------------------------------
	 double radtodeg = 57.29577951;
	 double L = (psGPS_PositionSlave->dLongitude - psGPS_PositionMaster->dLongitude)/radtodeg;  // degrees to rad
	 double U1 = atan( (1-f) * tan( ( psGPS_PositionMaster->dLatitude/radtodeg) ) );
	 double U2 = atan( (1-f) * tan( ( psGPS_PositionSlave->dLatitude /radtodeg) ) );
	 double sinU1 = sin(U1);
	 double sinU2 = sin(U2);
	 double cosU1 = cos(U1);
	 double cosU2 = cos(U2);
	 //---------------------------------------------------------------------
	 double lambda = L;
	 double lambdaP = 0.0;
	 uint32_t iterLimit = 100;
	 double cosSqAlpha = 0;
	 double cos2SigmaM = 0;
	 double cosSigma = 0;
	 double sigma = 0;
	 double sinSigma = 0;
	 double sinLambda = 0;
	 double cosLambda = 0;

	 do
	 {
		 sinLambda = sin(lambda);
		 cosLambda = cos(lambda);
		 sinSigma  = sqrt( (cosU2*sinLambda) * (cosU2*sinLambda) +
		 (cosU1*sinU2-sinU1*cosU2*cosLambda) * (cosU1*sinU2-sinU1*cosU2*cosLambda));
		 //-- UnSafe comparing two float values -
		 if ( 0.0 == sinSigma )
		 {
			 psCOMP_Data->dDistance = 0.0;
			 psCOMP_Data->dBear     = 0.0;
			 return (0);; // co-incident points
		 }
		 
		 cosSigma = sinU1*sinU2 + cosU1*cosU2*cosLambda;
		 sigma = atan2(sinSigma, cosSigma);
		 double sinAlpha = cosU1 * cosU2 * sinLambda / sinSigma;
		 cosSqAlpha = 1 - sinAlpha*sinAlpha;
		 cos2SigmaM = cosSigma - 2*sinU1*sinU2/cosSqAlpha;
		 if (isnan(cos2SigmaM)) cos2SigmaM = 0;  // equatorial line: cosSqAlpha=0 (§6)
		 double C = f/16*cosSqAlpha*(4+f*(4-3*cosSqAlpha));
		 lambdaP = lambda;
		 lambda = L + (1-C) * f * sinAlpha *
		 (sigma + C*sinSigma*(cos2SigmaM+C*cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)));
	 } while ( fabs(lambda - lambdaP) > 1e-12 && --iterLimit > 0);

	 if ( 0 == iterLimit ) return 0 ;  // formula failed to converge

	 double uSq = cosSqAlpha * (a*a - b*b) / (b*b);
	 double A = 1 + uSq/16384*(4096+uSq*(-768+uSq*(320-175*uSq)));
	 double B = uSq/1024 * (256+uSq*(-128+uSq*(74-47*uSq)));
	 double deltaSigma = B*sinSigma*(cos2SigmaM+B/4*(cosSigma*(-1+2*cos2SigmaM*cos2SigmaM)-
	 B/6*cos2SigmaM*(-3+4*sinSigma*sinSigma)*(-3+4*cos2SigmaM*cos2SigmaM)));
	 
	 psCOMP_Data->dDistance = b*A*(sigma-deltaSigma);
	 psCOMP_Data->dBear = atan2( cosU2*sinLambda, (cosU1*sinU2-sinU1*cosU2*cosLambda)) * radtodeg;
	 
	 if(psCOMP_Data->dBear<0) psCOMP_Data->dBear+=360;
	 //psCOMP_Data->dBear = psCOMP_Data->dBear - atan2((psGPS_PositionSlave->dLatitude - psGPS_PositionMaster->dLatitude),(psGPS_PositionSlave->dLongitude - psGPS_PositionMaster->dLongitude)) * radtodeg;
	 

	 return(0);
 }
 
 


void GPRMRC_Decode(char *gprmc,GPS_Queue *info)
{	
	for(uint8_t i=0;i<6;i++)
	{
		info->NMEA[i]=gprmc[i];
	}
	char La[12];
	char Lo[13];
	/*getting info->DataToGPS.Time*/
	for(uint8_t i=0;i<6;i++)
	{
		info->Time[i]=gprmc[i+7];
	}
	info->Time[8]=0;
	info->Time[7]=info->Time[5];
	info->Time[6]=info->Time[4];
	info->Time[5]=':';
	info->Time[4]=info->Time[3];
	info->Time[3]=info->Time[2];
	info->Time[2]=':';
	
	
	/*getting Latitude*/
	for(uint8_t i=0;i<11;i++)
	{
		La[i]=gprmc[i+20];
		Lo[i]=gprmc[i+32];
	}
	
	// ddmm.mmmmm,dddmm.mmmm
	uint32_t dwDegrees = 0;
	dwDegrees = (La[0]-'0')*10*60*10000  + (La[1]-'0')*60*10000 +	(La[2]-'0')*100000        + (La[3]-'0')*10000 +
	(La[5]-'0')*1000          + (La[6]-'0')*100 +
	(La[7]-'0')*10            + (La[8]-'0');
	
	info->dLatitude = ((double)dwDegrees/(60*10000));
	
 	dwDegrees = 0;
 	dwDegrees = (Lo[0]-'0')*10*10*60*10000 + (Lo[1]-'0')*10*60*10000  + (Lo[2]-'0')*60*10000 + 	(Lo[3]-'0')*100000 + (Lo[4]-'0')*10000 +
 	(Lo[6]-'0')*1000   + (Lo[7]-'0')*100    +
 	(Lo[8]-'0')*10     + (Lo[9]-'0');
 	
 	info->dLongitude = ((double)dwDegrees/(60*10000));
	
	for(uint8_t i=0;i<4;i++)
	{	
		if(i!=1)info->Speed_knots[i]=gprmc[i+45];
		
	}
		info->Speed_knots[4]=0;
	
	
	// 	info->DataToGPS.Latitude[9]='\n';
	// 	info->DataToGPS.Longtitude[9]='\n';

}

void Coordinates_calc(char *La,char *Lo,GPS_POSITION_t *Para)
{
	// ddmm.mmmmm,dddmm.mmmm
	uint32_t dwDegrees = 0;
	dwDegrees = (La[0]-'0')*10*60*10000  + (La[1]-'0')*60*10000 +	(La[2]-'0')*100000        + (La[3]-'0')*10000 +
	(La[5]-'0')*1000          + (La[6]-'0')*100 +
	(La[7]-'0')*10            + (La[8]-'0');
	
	Para->dLatitude = ((double)dwDegrees/(60*10000));
	
	dwDegrees = 0;
	dwDegrees = (Lo[0]-'0')*10*10*60*10000 + (Lo[1]-'0')*10*60*10000  + (Lo[2]-'0')*60*10000 + 	(Lo[3]-'0')*100000 + (Lo[4]-'0')*10000 +
	(Lo[6]-'0')*1000   + (Lo[7]-'0')*100    +
	(Lo[8]-'0')*10     + (Lo[9]-'0');
	
	Para->dLongitude = ((double)dwDegrees/(60*10000));
	
}

/***********************************************/
void GPS_Task(void *pvParameters)
{	
	LCD_Queue LCD;
	GPS_Queue DataToGPS;
	GPS_POSITION_t sGPS_MasterPosition;
	GP_GGA	GPS_data_decode;
	uint8_t Message=0;
	
	Gps_init();

	NVIC_DisableIRQ((IRQn_Type)USART1_IRQn);
	NVIC_ClearPendingIRQ((IRQn_Type) USART1_IRQn);
	NVIC_EnableIRQ((IRQn_Type)USART1_IRQn);
	NVIC_SetPriority((IRQn_Type)USART1_IRQn,configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(USART1_IRQn);
 	usart_enable_interrupt(GPS_USART,US_IER_RXRDY);
 	
	/*init position */
	static GPS_POSITION_t Master_Last= {50.0036792, 16.240169};
	static GPS_POSITION_t Slave_Last= {50.0036792, 16.240169};		
	GPS_COMP_DATA_t sCOMP_Data;
	float HDOP=0;
	uint32_t temp_HDOP=0;
	uint32_t temp_GPS_ALT=0;
	
	
	Senzor_Queue Data_Queue_GPS;
	
	
	for (;;)
	{	
						
		if(xQueueReceive(Queue_GPS,&DataToGPS,1500)==pdPASS)//po 100 ticích se obsah zahodí - je prázdná  fronta
		{	
			
			//Pokud Byla nalezena veta
			if (DataToGPS.GL_flag_gps==SENTENCE_OK)
			{
				DataToGPS.Nmea_crc[0]=DataToGPS.Nmea_data[strlen(DataToGPS.Nmea_data)-MSB_CRC_CONST];
				DataToGPS.Nmea_crc[1]=DataToGPS.Nmea_data[strlen(DataToGPS.Nmea_data)-LSB_CRC_CONST];
				
				DataToGPS.Nmea_crc[0]=(unsigned char)(myAtoi(&DataToGPS.Nmea_crc[0])<<4);
				DataToGPS.Nmea_crc[0]|=(unsigned char)myAtoi(&DataToGPS.Nmea_crc[1]);
				
				DataToGPS.My_crc=Calculate_CRC(&DataToGPS.Nmea_data[0],(strlen(DataToGPS.Nmea_data)-UNTIL_CRC_CONST));	//mnou spoèítaný CRC
				
				if((DataToGPS.My_crc==DataToGPS.Nmea_crc[0]))
				{
					//Decode GPS sentence
					Message=Nmea_decode(DataToGPS.Nmea_data);
		
					switch(Message)
					{
						case GPGGA:
							GPGGA_Decode(&DataToGPS,&GPS_data_decode);
							Coordinates_calc(&GPS_data_decode.Latitude,&GPS_data_decode.Longitude,&Slave_Last);
							GPS_Utils_CalcDisAndBear(&Master_Last,&Slave_Last,&sCOMP_Data);
							temp_HDOP=(GPS_data_decode.HDOP[0]-'0')*100+(GPS_data_decode.HDOP[2]-'0')*10+(GPS_data_decode.HDOP[3]-'0');
							HDOP=(float)temp_HDOP/100;
 							if (HDOP<2)
 							{
								 ioport_toggle_pin_level(LED0);
 							}
							
							temp_GPS_ALT=(GPS_data_decode.Altitude[0]-'0')*1000+(GPS_data_decode.Altitude[1]-'0')*100+(GPS_data_decode.Altitude[2]-'0')*10+(GPS_data_decode.Altitude[4]-'0');
							//temp_GPS_ALT/=10;
							
							Data_Queue_GPS.gps_alt=(float)(temp_GPS_ALT/10);
 							Data_Queue_GPS.senzor_type=GPS_TYPE;
// 							Data_Queue_GPS.gps_alt[0]=(GPS_data_decode.Altitude[0]-'0');
// 							Data_Queue_GPS.gps_alt[1]=(GPS_data_decode.Altitude[1]-'0');
// 							Data_Queue_GPS.gps_alt[2]=(GPS_data_decode.Altitude[2]-'0');
// 							Data_Queue_GPS.gps_alt[3]=(GPS_data_decode.Altitude[4]-'0');
							xQueueSend(Queue_Senzor_Task,&Data_Queue_GPS,portMAX_DELAY);
							
							break;
						
						case GPTXT:
						
							break;
						
						case INVALID:
											
							//xSemaphoreGive(Lights_LCD);
						
							break;
						
						case NO_SENTENCE:
						
							break;
						
						default: break;
					}
					
				}else  //nesedi CRC
				{
					
				}
				
			}//nalezena veta sentence OK
			
		}
	}

}





