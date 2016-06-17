/*
 * GPS_Driver.c
 *
 * Created: 7.7.2014 12:01:09
 *  Author: JR
 */ 

#include <asf.h>
#include <stdlib.h>
#include "FreeRTOS_V_8_Header.h"
#include "string.h"
#include "GPS_Driver.h"
#include "GPS_Task.h"

/***********************************************************************/
uint8_t Calculate_CRC(uint8_t *crc, uint8_t length)
{
	uint8_t CR=0;
	
	for(uint8_t i=1;i<length;i++)
	{
		CR^=crc[i];
	}
	
	return CR;
}

/*************************************************/
void Gps_init(void)
{
	
	uint8_t MY_CRC=0;
	
  	vTaskDelay(10/portTICK_RATE_MS);
  	ioport_set_pin_level(GPS_RESET_PIN,false);
  	vTaskDelay(500/portTICK_RATE_MS);
 	ioport_set_pin_level(GPS_RESET_PIN,true);

//   	MY_CRC=Calculate_CRC(Fix_ctl,strlen(default_setting)-5);//-5 pro 0x29
//  	 
//   	 if (MY_CRC==0)
//    	 {
//    		 delay_ms(10);
//    	 }
	
	http://www.hhhh.org/wiml/proj/nmeaxor.html
 	vTaskDelay(500/portTICK_RATE_MS);
 	usart_serial_write_packet(GPS_USART,PMTK_SET_NMEA_OUTPUT_GGAONLY,strlen(PMTK_SET_NMEA_OUTPUT_GGAONLY));
  	vTaskDelay(100/portTICK_RATE_MS);
	usart_serial_write_packet(GPS_USART,PMTK_SET_NMEA_UPDATE_5HZ,strlen(PMTK_SET_NMEA_UPDATE_5HZ));
  	vTaskDelay(100/portTICK_RATE_MS);
	usart_serial_write_packet(GPS_USART,PMTK_API_SET_FIX_CTL_5HZ,strlen(PMTK_API_SET_FIX_CTL_5HZ));
	vTaskDelay(100/portTICK_RATE_MS);
  	
//   	usart_serial_write_packet(GPS_USART,GLL_OFF,strlen(GLL_OFF));
//   	vTaskDelay(100/portTICK_RATE_MS);
//   	
//   	usart_serial_write_packet(GPS_USART,GSA_OFF,strlen(GSA_OFF));
//   	vTaskDelay(100/portTICK_RATE_MS);
//   	
//   	usart_serial_write_packet(GPS_USART,GSV_OFF,strlen(GSV_OFF));
	  
	  
	 
//   	vTaskDelay(100/portTICK_RATE_MS);
//  	usart_serial_write_packet(GPS_USART,PMTK_API_SET_FIX_CTL_5HZ,strlen(PMTK_API_SET_FIX_CTL_5HZ));
//  
	
		
//	}
	//delay_ms(2000);
	//usart_serial_write_packet(CONF_UART,Fix_ctl,strlen(Fix_ctl));
	//delay_ms(1000);

	
	
	
}

/***********************************************/

void GPGGA_Decode(GPS_Queue *DataToGPS,GP_GGA *S_GPGGA)
{	
	uint8_t pole_carek[80];
	uint8_t Temp=0;
	
	for (uint8_t i=0;i<strlen(DataToGPS->Nmea_data);i++)
	{
		if (DataToGPS->Nmea_data[i]==',')
		{	
			pole_carek[Temp]=i;
			Temp++;
		}
		
	}
	
	//UTC Time
	S_GPGGA->Time[0]=DataToGPS->Nmea_data[GGA_TIME_OFFSET];
	S_GPGGA->Time[1]=DataToGPS->Nmea_data[1+GGA_TIME_OFFSET];
	S_GPGGA->Time[2]=':';
	S_GPGGA->Time[3]=DataToGPS->Nmea_data[2+GGA_TIME_OFFSET];
	S_GPGGA->Time[4]=DataToGPS->Nmea_data[3+GGA_TIME_OFFSET];
	S_GPGGA->Time[5]=':';
	S_GPGGA->Time[6]=DataToGPS->Nmea_data[4+GGA_TIME_OFFSET];
	S_GPGGA->Time[7]=DataToGPS->Nmea_data[5+GGA_TIME_OFFSET];
	S_GPGGA->Time[8]=0;	
	
	//Data Valid
	//DataToGPS->Data_valid=DataToGPS->Time[7]=DataToGPS->Nmea_data[7+GGA_TIME_OFFSET];
		
	uint8_t j=0;
	//Latitude - za druhou carkou
	do 
	{	
		j++;		
		S_GPGGA->Latitude[j-1]=DataToGPS->Nmea_data[j+pole_carek[1]];
		
	} while ((DataToGPS->Nmea_data[j+pole_carek[1]]!='N')&&(DataToGPS->Nmea_data[j+pole_carek[1]]!='S'));
		
	S_GPGGA->Latitude[j]=0;
		
	//Longitude
	j=0;
	do
	{
		j++;
		S_GPGGA->Longitude[j-1]=DataToGPS->Nmea_data[j+pole_carek[3]];
		
	} while ((DataToGPS->Nmea_data[j+pole_carek[3]]!='E')&&(DataToGPS->Nmea_data[j+pole_carek[3]]!='W'));
	
	S_GPGGA->Longitude[j]=0;
			
	//Number of satelites being used (0 ~ 12)
	j=0;
	do
	{
		j++;
		S_GPGGA->Sat_used[j-1]=DataToGPS->Nmea_data[j+pole_carek[6]];
		
	} while (DataToGPS->Nmea_data[j+1+pole_carek[6]]!=',');
	
	S_GPGGA->Sat_used[j]=0;
		
	//HDOP
	j=0;
	do
	{
		j++;
		S_GPGGA->HDOP[j-1]=DataToGPS->Nmea_data[j+pole_carek[7]];
		
	} while (DataToGPS->Nmea_data[j+1+pole_carek[7]]!=',');
	
	S_GPGGA->HDOP[j]=0;
	
	//ALTITUDE
	j=0;
	do
	{
		j++;
		S_GPGGA->Altitude[j-1]=DataToGPS->Nmea_data[j+pole_carek[8]];
		
	} while (DataToGPS->Nmea_data[j+1+pole_carek[8]]!=',');
	
	S_GPGGA->Altitude[j]=0;
	
	S_GPGGA->Konec_pole=0;
	
}

uint8_t Nmea_decode(uint8_t *str1)
{
	//kontorla na carky
	uint8_t Comma[2];
	uint8_t Temp=0;
	uint8_t delka=strlen(str1)-7; //-7 GGA muze mit na konci 2 carky za sebou
	
	for (uint8_t i=0;i<delka;i++)
	{
		Comma[i-Temp]=str1[i];
		
		if ((Comma[0]==',')&&(Comma[1]==','))
		{	
			
			return INVALID;	//Nenalezen signal GPS - jsou minimalne 2 carky vedle sebe
		}
		
		if (((i%2)!=0)&&(i!=0)) Temp+=2;
		
	}
			
	if (str_compare(GPS_GPRMC,str1)==0)
	{	
		
		//ioport_toggle_pin_level(LED0);	
		return GPRMC;
	}
	else if (str_compare(GPS_GPVTG,str1)==0)
	{
		return GPVTG;
	}
	else if (str_compare(GPS_GPGGA,str1)==0)
	{	
	//	ioport_toggle_pin_level(LED0);	
		
		return GPGGA;
		
	}
	else if (str_compare(GPS_GPGSA,str1)==0)
	{
		return GPGSA;
		
	}
	else if (str_compare(GPS_GPGSV,str1)==0)
	{
		return GPGSV;
		
	}
	else if (str_compare(GPS_GPGLL,str1)==0)
	{
		return GPGLL;
		
	}
	else if (str_compare(GPS_GPTXT,str1)==0)
	{
		return GPTXT;
		
	}else
	{
		return NO_SENTENCE;
	}
	
	
}

/***********************************************/
int myAtoi(unsigned char *str)
{
	int res = 0; // Initialize result
	
	// Iterate through all characters of input string and update result
	for (int i = 0; i <1; i++)
	{
		if (str[i]<='9')
		{
			res = + str[i] - '0';
		}else
		{
			str[i]-=(55);
			res=str[i];
		}
		
		
	}
	
	// return result.
	return res;
}


uint8_t str_compare(uint8_t s1[], uint8_t s2[])
{
	
	int i = 0;
	while ( s1[i] != '\0' )
	{
		if( s2[i] == '\0' ) { return 1; }
		else if( s1[i] < s2[i] ) { return -1; }
		else if( s1[i] > s2[i] ) { return 1; }
		i++;
	}
	return 0;

}