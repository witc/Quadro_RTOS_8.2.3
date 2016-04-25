/*
 * main.h
 *
 * Created: 19.2.2014 23:52:12
 *  Author: JR
 */ 


#ifndef MAIN_H_
#define MAIN_H_

#define BOARD_ID_USART             ID_UART1
#define BOARD_USART                UART1
#define BOARD_USART_BAUDRATE       19200

/* define mode of mpu */
#define FIFO_MPU9150		0
#define RAW_MPU9150			0
#define RAW_INT_MPU9150		1

/* define TX or RX */
#define	TX_TO_MATLAB		0
#define	RX_NEW_CMD			1

//LoRa
typedef enum
{
	RFLR_STATE_IDLE,
	RFLR_STATE_RX_INIT,
	RFLR_STATE_RX_RUNNING,
	RFLR_STATE_RX_DONE,
	RFLR_STATE_TX_INIT,
	RFLR_STATE_TX_RUNNING,
	RFLR_STATE_TX_DONE,
	RFLR_STATE_RX_TIMEOUT,
	RFLR_STATE_CAD_INIT,
	RFLR_STATE_CAD_RUNNING,
	RFLR_STATE_CRC_ERROR,
	RFLR_STATE_ERROR,
}RF_States;

typedef enum
{
	DIO_RESERVE,
	DIO0,
	DIO1,
	DIO2,
	DIO3,
}DIO_States;

typedef enum
{
	GPS_i,
	RF_i,
	COMPASS_i,
	LCD_i,
	DISATNCE_i,
	NOTE_i,
	LAST_COM_i,
	
}LCD_States;


#define	STATE_ON	200
#define STATE_OFF	201
#define CHANGE_STATE 202
#define STAY_IN_STATE 203

#define RDY_TO_SLEEP	210
//#define RDY_TO_SLEEP	210
	
/************************************************************************/
//Mng
typedef struct{
	
	short	Cmd;
	short	Data_State;
	
}MANAGE_TASK;

/********************************************************/
//GPS
typedef struct{
	char Time[9];
	//char Data_valid;
	char Latitude[12];
	char Longitude[13];
	char Sat_used[3];
	char HDOP[5];
	char Altitude[7];
	char id_psa;
	char Konec_pole;
	
}GP_GGA;

/************************************************************************/


/************************************************************************/
//RF
typedef struct{
	
	short Rssi;
	uint8_t Buffer[100];
	char AGC;
	MANAGE_TASK	Stat;
	
	//float Temp;
}RF_Queue;

typedef enum{
	NONE,
	FROM_TX,
	FROM_SENZOR,
}cmd_types;
	
//Motor
typedef struct{
	
	float pitch;
	float roll;
	float yaw;
	float Baro;
	short Gyro_d[3];
	
	uint16_t TX_CH_xx[4];
	uint8_t type_of_data;
	
	//float Temp;
}Motor_Queue;



#define MPU_TYPE	0x1
#define BARO_TYPE	0x2
#define GPS_TYPE	0x3
#define MAG_TYPE	0x4
/* MPU9150*/
typedef struct{
	
	uint8_t senzor_type;
	
#if (RAW_MPU9150==1)
	uint8_t MPU_FIFO[14];

#elif ((RAW_INT_MPU9150==1))

	uint8_t MPU_FIFO[14];
	uint8_t Vycti_data;
	
#elif (FIFO_MPU9150==1)

	uint8_t MPU_FIFO[1024];
	
#else
# error "Please specify Way to get a datta from MPU9150"
#endif	
	
	uint8_t baro[5];
	uint8_t mag[6];
	short temp;
	

}Senzor_Queue;

#if ((RAW_INT_MPU9150==1))

typedef struct{
	
	uint8_t MPU_FIFO[14];
	
}MPU9150_Buffer;
	
#endif

/************************************************************************/
//LCD

/**********************************/

typedef struct
{
	short Min_X;
	short Min_Y;
	short Min_Z;
	
	short Max_X;
	short Max_Y;
	short Max_Z;
	
	float Heading;
	short Calib;
	
}LCD_Magnetometer;

/**********************************/
typedef struct
{
	uint8_t Time[9];
	uint8_t Lat[12];
	uint8_t Long[14];
	uint8_t HDOP[4];
	uint8_t Sat[2];
	short  RSSI;
	
}RF_RX;
/**********************************/

typedef struct
{
	char Message[100];
}LCD_Notification;

/**********************************/
typedef struct
{
	short   GPS_status[3];	//3=master
	
	short	id_psa;
	long	Distance[2];
	short	Bear[2];
	short	Last_Com[3];	//3==master
}LCD_Distance;


typedef struct
{
	char	Task_id;
	MANAGE_TASK	Stat;
	RF_RX   RF_RX_Data;
	GP_GGA	Sentence;
	LCD_Distance Dist;
	LCD_Notification Note;
	LCD_Magnetometer Mag;
	
}LCD_Queue;



#define NO_GPS	0x55
#define OK_GPS	0x56
#define ATIME			0x57

#define CRC_OK			0x63
#define CRC_FALSE		0x64

#define TIMER_RUN	0x3366
#define NEW_PACKET	0x3399


#endif /* MAIN_H_ */