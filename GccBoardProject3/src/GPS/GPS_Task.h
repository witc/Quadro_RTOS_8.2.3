/*
 * GPS_Task.h
 *
 * Created: 27.1.2014 11:38:14
 *  Author: JR
 */ 


#ifndef GPS_TASK_H_
#define GPS_TASK_H_


typedef struct{
	char Time[9];	//vsude je o jedno navic - aby se nespojily ty pole do jednoho, prazdy chlivek oddeli pole
	char Latitude[12];
	char Longtitude[12];
	char Speed_knots[7];
	float Speed_km;
	char Nmea_data[100];
	char GL_flag_gps;
	unsigned char Nmea_crc[3];
	unsigned char My_crc;
	short err_count;
	double dLongitude;
	double dLatitude;
	char NMEA[8];
	
}GPS_Queue;

typedef struct{
	double dLatitude;
	double dLongitude;
	
} GPS_POSITION_t;

typedef struct{
	double dDistance;
	double dBear;

} GPS_COMP_DATA_t;

typedef enum {SENTENCE_OK,SENTENCE_FALSE} 	GPS_SENTENCE_ENUM;

#define GPS_GPRMC    "$GPRMC"
#define GPS_GPGLL    "$GPGLL"
#define GPS_GPTXT    "$GPTXT"

#define MSB_CRC_CONST	3
#define LSB_CRC_CONST	2
#define UNTIL_CRC_CONST	4


void GPRMRC_Decode(char *gprmc,GPS_Queue *info);
void GPS_Task(void *pvParameters);
void Dma_init(void);
void Coordinates_calc(char *La,char *Lo,GPS_POSITION_t *Para);
char GPS_Utils_CalcDisAndBear( GPS_POSITION_t* psGPS_PositionMaster, GPS_POSITION_t* psGPS_PositionSlave, GPS_COMP_DATA_t* psCOMP_Data );










#endif /* GPS_TASK_H_ */