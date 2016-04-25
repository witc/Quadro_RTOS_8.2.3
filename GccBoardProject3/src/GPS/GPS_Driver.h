/*
 * GPS_Driver.h
 *
 * Created: 7.7.2014 12:01:26
 *  Author: JR
 */ 


#ifndef GPS_DRIVER_H_
#define GPS_DRIVER_H_

#include "Main.h"
#include "GPS_Task.h"


uint8_t Nmea_decode(uint8_t *str1);
uint8_t Calculate_CRC(uint8_t *crc, uint8_t length);
int myAtoi(char *str);
uint8_t str_compare(uint8_t s1[], uint8_t s2[]);

void Gps_init(void);
void Gps_Sleep(void);
void Gps_WakeUp(void);

void GPRMC_Decode(GPS_Queue *DataToGPS);
void GPVTG_Decode(GPS_Queue *DataToGPS);
void GPGGA_Decode(GPS_Queue *DataToGPS,GP_GGA *S_GPGGA);
void GPGSA_Decode(GPS_Queue *DataToGPS);
void GPGSV_Decode(GPS_Queue *DataToGPS);
void GPGLL_Decode(GPS_Queue *DataToGPS);
void GPTXT_Decode(GPS_Queue *DataToGPS);


typedef enum {GPRMC,GPVTG,GPGGA,GPGSA,GPGSV,GPGLL,GPTXT,INVALID,NO_SENTENCE} 	GPS_MESSAGE_ENUM;

#define GPS_GPRMC    "$GPRMC"
#define GPS_GPVTG    "$GPVTG"
#define GPS_GPGGA    "$GPGGA"
#define GPS_GPGSA    "$GPGSA"
#define GPS_GPGSV    "$GPGSV"
#define GPS_GPGLL    "$GPGLL"
#define GPS_GPTXT    "$GPTXT"


#define GGA_TIME_OFFSET	7
#define GGA_LATI_OFFSET	18
#define GGA_LONG_OFFSET	30	//?31?
#define GGA_NSAT_OFFSET	45	//46?
#define GGA_HDOP_OFFSET	48	//5?
#define GGA_ALTI_OFFSET	52	//54?


// different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
// Note that these only control the rate at which the position is echoed, to actually speed up the
// position fix you must also send one of the position fix rate commands below too.
#define PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ  "$PMTK220,10000*2F\r\n" // Once every 10 seconds, 100 millihertz.
#define PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ  "$PMTK220,5000*1B\r\n"  // Once every 5 seconds, 200 millihertz.
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F\r\n"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C\r\n"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r\n"
// Position fix update rate commands.
#define PMTK_API_SET_FIX_CTL_100_MILLIHERTZ  "$PMTK300,10000,0,0,0,0*2C\r\n" // Once every 10 seconds, 100 millihertz.
#define PMTK_API_SET_FIX_CTL_200_MILLIHERTZ  "$PMTK300,5000,0,0,0,0*18\r\n"  // Once every 5 seconds, 200 millihertz.
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C\r\n"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F\r\n"
// Can't fix position faster than 5 times a second!


#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C\r\n"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17\r\n"

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"//49 za 29
// turn on only GPGGA
#define PMTK_SET_NMEA_OUTPUT_GGAONLY "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"//49 za 29
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"

// to generate your own sentences, check out the MTK command datasheet and use a checksum calculator
// such as the awesome http://www.hhhh.org/wiml/proj/nmeaxor.html


#define MAESTRO_GPS_5HZ "$PSRF103,00,6,00,0*23\r\n"
#define MAESTRO_GPS_1HZ "$PSRF103,00,7,00,0*22\r\n"

#define MAESTRO_GPS_RMC " $PSRF103,04,00,01,01*21\r\n"

#define GGA_ON   "$PSRF103,00,00,01,01*25\r\n"
#define GGA_OFF  "$PSRF103,00,00,00,01*24\r\n"

// GLL-Geographic Position-Latitude/Longitude, message 103,01
#define LOG_GLL 0
#define GLL_ON   "$PSRF103,01,00,01,01*26\r\n"
#define GLL_OFF  "$PSRF103,01,00,00,01*27\r\n"

// GSA-GNSS DOP and Active Satellites, message 103,02
#define LOG_GSA 0
#define GSA_ON   "$PSRF103,02,00,01,01*27\r\n"
#define GSA_OFF  "$PSRF103,02,00,00,01*26\r\n"

// GSV-GNSS Satellites in View, message 103,03
#define LOG_GSV 0
#define GSV_ON   "$PSRF103,03,00,01,01*26\r\n"
#define GSV_OFF  "$PSRF103,03,00,00,01*27\r\n"

// RMC-Recommended Minimum Specific GNSS Data, message 103,04
#define LOG_RMC 1
#define RMC_ON   "$PSRF103,04,00,01,01*21\r\n"
#define RMC_OFF  "$PSRF103,04,00,00,01*20\r\n"

#endif /* GPS_DRIVER_H_ */