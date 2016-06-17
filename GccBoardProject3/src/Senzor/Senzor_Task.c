/*
 * Senzor_Task.c
 *
 * Created: 27.1.2014 21:45:14
 *  Author: JR
 */ 

#include <asf.h>
#include <math.h>
#include <arm_math.h>
#include "FreeRTOS.h"
#include "main.h"
#include "Senzor_Task.h"
#include "MPU_9150.h"
#include "FreeRTOS_V_8_Header.h"
#include "Periph_init.h"
#include "MPL3115A2.h"
#include "Mag.h"
#include "Motor.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "MadgwickAHRS.h"
#include "Imu.h"
#include "MS5611.h"

/** PWM frequency in Hz */

#define PWM_FREQUENCY      500
/** Period value of PWM output waveform */
#define PERIOD_VALUE       1000
/** Initial duty cycle value */

#define INIT_DUTY_VALUE    50


#define USE_MARG_AHRS	1

extern	volatile	xQueueHandle		Queue_RF_Task;
extern volatile	 xQueueHandle		Queue_Senzor_Task;
extern volatile	xQueueHandle		Queue_Motor_Task;

volatile xTimerHandle Baro_Timer;
volatile xTimerHandle Mag_Timer;


extern Pdc *twi_dma_inst;
extern pdc_packet_t twi_dma_packet;
uint8_t GL_buffer_mpu_9150[14];
/* offset for Gyro */
volatile short offset[3]={0,0,0};
extern int32_t accSum[3];
extern	int16_t accSmooth[3];
extern float accAlt, vel;
	//volatile	MPU9150_Buffer XYZ;
	
void Gyro_Angle(short g_x,short g_y,short g_z,EulerAngles *uhly,float dt);
void Akce_Angle(short a_x, short a_y, short a_z,EulerAngles *uhly,float dt);
void Fir(float *data,short id);

#define FIR 22
const float Fir_HPass[22]=  {
	-0.002747379929816, 0.003638146822209, 0.001290138590684, -0.01280559110698,
	0.02090462382392,-0.008970485586287, -0.02951470247043,  0.07349565726954,
	-0.07288680587089, -0.04333934189475,   0.5701777796257,   0.5701777796257,
	-0.04333934189475, -0.07288680587089,  0.07349565726954, -0.02951470247043,
	-0.008970485586287,  0.02090462382392, -0.01280559110698, 0.001290138590684,
	0.003638146822209,-0.002747379929816
};



void Mag_TimerCallback(xTimerHandle pxTimer)
{
	Senzor_Queue Data_Queue_MAG;
	signed portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken=pdTRUE;	//pøerušení se dokonèí celé= pdFalse
	
	
// 	if(Mag_get_b(Data_Queue_MAG.mag)==0)
// 	{
		Data_Queue_MAG.senzor_type=MAG_TYPE;
		//Data_Queue_BARO.Vycti_data=1;
		xQueueSendToBackFromISR(Queue_Senzor_Task,&Data_Queue_MAG,&xHigherPriorityTaskWoken);
		//pio_enable_interrupt(PIOB, PIO_PB0);
		
		if (xHigherPriorityTaskWoken==pdTRUE) portYIELD();
		
	//}
}

void Baro_TimerCallback(xTimerHandle pxTimer)
{	
	Senzor_Queue Data_Queue_BARO;
	signed portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken=pdTRUE;	//pøerušení se dokonèí celé= pdFalse
	uint8_t data=0;
	static bool state=0;
	
	static uint32_t uT=0;
	static uint32_t uP=0;
	
	if (state==0)
	{	
		state=1;
		uP=ms5611_get_up();
		ms5611_start_ut();
		
	}else
	{
		uT=ms5611_get_ut();
		ms5611_start_up();
		state=0;
	}
	
	Data_Queue_BARO.baro_uP=uP;
	Data_Queue_BARO.baro_uT=uT;
	Data_Queue_BARO.senzor_type=BARO_TYPE;
	//Data_Queue_BARO.Vycti_data=1;
	xQueueSendToBackFromISR(Queue_Senzor_Task,&Data_Queue_BARO,&xHigherPriorityTaskWoken);
	//pio_enable_interrupt(PIOB, PIO_PB0);
	
	if (xHigherPriorityTaskWoken==pdTRUE) portYIELD();
	
	
// 	if(Baro_get_preasure(Data_Queue_BARO.baro)==0)
// 	{	
// 		Data_Queue_BARO.senzor_type=BARO_TYPE;
// 	 	//Data_Queue_BARO.Vycti_data=1;
// 	 	xQueueSendToBackFromISR(Queue_Senzor_Task,&Data_Queue_BARO,&xHigherPriorityTaskWoken);
// 	 	//pio_enable_interrupt(PIOB, PIO_PB0);
// 	 	
// 	 	if (xHigherPriorityTaskWoken==pdTRUE) portYIELD();
// 		 
// 	}
}



void TWI0_Handler(void)
{	
// 	MPU9150_Queue Data_Queue_MPU;
// 	signed portBASE_TYPE xHigherPriorityTaskWoken;
// 	xHigherPriorityTaskWoken=pdFALSE;	//pøerušení se dokonèí celé= pdFalse
// 	
// 	uint32_t status=0;
// 	status=twi_get_interrupt_status(TWI0);
// 	/* Get UART status and check if PDC receive buffer is full */
// 	if ((status & TWI_SR_TXCOMP) == TWI_SR_TXCOMP)
// 	{
// 		/* Initialize PDC data packet for transfer */
// 		twi_dma_packet.ul_addr = (uint32_t) GL_buffer_mpu_9150;
// 		twi_dma_packet.ul_size = 14;
// 		/* Configure PDC for data receive */
// 		pdc_rx_init(twi_dma_inst, &twi_dma_packet, NULL);
// 		/* Enable PDC transfers */
// 		pdc_enable_transfer(twi_dma_inst,  PERIPH_PTCR_RXTEN);
		twi_enable_interrupt(TWI0, TWI_IER_RXBUFF);
// 	}
// 	else if((status & TWI_SR_RXBUFF) == TWI_SR_RXBUFF)
// 	{
// 		for (uint8_t i=0;i<14;i++)
// 		{
// 			Data_Queue_MPU.MPU_FIFO[i]=GL_buffer_mpu_9150[i];
// 		}
// 		Data_Queue_MPU.Vycti_data=1;		
// 		xQueueSendToBackFromISR(Queue_Senzor_Task,&Data_Queue_MPU,&xHigherPriorityTaskWoken);
// 		
// 	}
}


void MPU9150_INT(void)
{	
	//NVIC_ClearPendingIRQ(PIOB_IRQn);
	Senzor_Queue Data_Queue_MPU;
	signed portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken=pdTRUE;	//pøerušení se dokonèí celé= pdFalse
	uint8_t data=0;
	
				
	//pio_disable_interrupt(PIOB, PIO_PB0);
//	pio_disable_interrupt(PIOA, PIO_PB0);
	
	//MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,Data_Queue_MPU.MPU_FIFO,MPU6050_RA_ACCEL_XOUT_H, 14);
// 	if(MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS,Data_Queue_MPU.MPU_FIFO,MPU6050_RA_ACCEL_XOUT_H, 14)!=TWI_SUCCESS)
// 	{
// 		usart_write_line((Usart*)UART1,"Reading buffer data false FALSE...\n");
// 		NVIC_SystemReset();
// 	}
	
	Data_Queue_MPU.senzor_type=MPU_TYPE;				
	Data_Queue_MPU.Vycti_data=1;
	xQueueSendToBackFromISR(Queue_Senzor_Task,&Data_Queue_MPU,&xHigherPriorityTaskWoken);
	//pio_enable_interrupt(PIOB, PIO_PB0);
	
	if (xHigherPriorityTaskWoken==pdTRUE) portYIELD();
    //MPU9150_getMotion_dma(NULL);
			
}

void Fir(float *data,short id)
{
	float Temp=0;
	static float Temp_pole[6][FIR];
	static short poc=0;
	
	for (short i=0;i<(FIR-1);i++)
	{
		Temp_pole[id][i]=Temp_pole[id][i+1];
	}
	
	Temp_pole[id][FIR-1]=*data;
	poc++;
	if (poc==10) poc=0;
	
	
	for (short j=0;j<FIR;j++)
	{
		Temp+=(float)(Fir_HPass[j]*(Temp_pole[id][j]));
	}
	
	*data=(float)Temp;
}

void Gyro_Angle(short g_x,short g_y,short g_z,EulerAngles *uhly, float dt)
{
	#define GAA			 2
	
	uint8_t			data_av=0;
	
	
	static short		pole1[GAA];
	static short		pole2[GAA];
	static short		pole3[GAA];
	
	static unsigned int	counter=0;

	
	static float	Alfa1=0;
	static float	Alfa2=0;
	
		
// 	Suma1+=x;
// 	pole1[counter]=x;
// 	Suma2+=y;
// 	pole2[counter]=y;
// 	Suma3+=z;
// 	pole3[counter]=z;
// 	counter++;
// 	
// 	if (counter==GAA) counter=0;
// 
// 	Suma1-=pole1[counter];
// 	Temp1=(Suma1/GAA);
// 	Suma2-=pole2[counter];
// 	Temp2=(Suma2/GAA);
// 	Suma3-=pole3[counter];
// 	Temp3=(Suma3/GAA);
		
	//Temp2=(Temp2/65535)*500;
	Alfa1+=(float)(g_x*0.06103515f*dt);
	Alfa2+=(float)(g_y*0.06103515f*dt);
	
	// 		Alfa1=Temp2;
	// 		Alfa2=Temp3;
	
	uhly->pitch=(float)(Alfa1);
	uhly->roll=(float)(Alfa2);
	
	//PORTC.OUTCLR=0b1000;
	

}


void Akce_Angle(short a_x, short a_y, short a_z,EulerAngles *uhly,float dt)
{
// 	#define AAA		25
// 		
// 	uint8_t				data_av=0;
// 	static int32_t		Suma1=0;
// 	static int32_t		Suma2=0;
// 	static int32_t		Suma3=0;
// 	
// 	static int16_t		pole1[AAA];
// 	static int16_t		pole2[AAA];
// 	static int16_t		pole3[AAA];
// 	
// 	static unsigned int	counter=0;
// 	double		Temp1 = 0;
// 	double		Temp2 = 0;
// 	double      Temp3 = 0;
// 	
// 	
// 	
// 	Suma1+=a_x;
// 	pole1[counter]=a_x;
// 	Suma2+=a_y;
// 	pole2[counter]=a_y;
// 	Suma3+=a_z;
// 	pole3[counter]=a_z;
// 	counter++;
// 
// 	if (counter==AAA) counter=0;
// 
// 	Suma1-=pole1[counter];
// 	Temp1=(Suma1/AAA);
// 	Suma2-=pole2[counter];
// 	Temp2=(Suma2/AAA);
// 	Suma3-=pole3[counter];
// 	Temp3=(Suma3/AAA);
	
	/* use LPF filter */
	float acc_smooth[3];
	acc_smooth[0]=filterApplyPt1((float)a_x,&Filters[ACC_X],20,dt);
	acc_smooth[1]=filterApplyPt1((float)a_y,&Filters[ACC_Y],20,dt);
	acc_smooth[2]=filterApplyPt1((float)a_z,&Filters[ACC_Z],20,dt);
	
	/* angle in RAD */
	uhly->pitch=(float)(float)((atan2((double)acc_smooth[1],(double)acc_smooth[2])));
	uhly->roll=(float)((-atan2((double)acc_smooth[0],(double)acc_smooth[2])));
	
	//calculateHeading
	//uhly->pitch=(float)((atan2((double)Temp2,(double)Temp3)*(180)/3.141f));
	//Alfa1-=90;
// 	if (Alfa1>90)  Alfa1=90;
// 	if (Alfa1<-90) Alfa1=-90;
	
	
	//uhly->pitch=(float)(Alfa1);
	
//	uhly->roll=(float)((-atan2((double)Temp1,(double)Temp3)*(180)/3.141f));
	

// 	float cosineRoll = cosf(anglerad[ROLL]);
// 	float sineRoll = sinf(anglerad[ROLL];)
// 	float cosinePitch = cosf(anglerad[PITCH]);
// 	float sinePitch = sinf(anglerad[PITCH]);
// 	float Xh = vec->A[X] * cosinePitch + vec->A[Y] * sineRoll * sinePitch + vec->A[Z] * sinePitch * cosineRoll;
// 	float Yh = vec->A[Y] * cosineRoll - vec->A[Z] * sineRoll;
// 	float hd = (atan2f(Yh, Xh) * 1800.0f / M_PI + magneticDeclination) / 10.0f;
// 	head = lrintf(hd);
// 	if (head < 0)
// 	head += 360;
// 
// 	return head;
		
}



void Senzor_Task(void *pvParameters)
{	
	Senzor_Queue Senzor;
	//MPU9150_Buffer XYZ;
	Motor_Queue Position;
	RF_Queue Semtech;
	
	 MAG_XYZ MAG;
 	EulerAngles Angles_A;
	portTickType CurrentTime=xTaskGetTickCount();
	portTickType CurrentTime_Baro=CurrentTime;
	portTickType LastTime=CurrentTime;
	portTickType LastTime_Baro=CurrentTime;
	
	#define CONST_FILTER 0.998f
		
	char Kalibrace=NULL;
	static float uhel[3];
	float f_temp[3];
	uint16_t packet_count=0;
	
	float baro_meas=0,last_baro_meas=0,temp_gps_alt;
	int32_t tlak,teplota;
	float baro_vel=0;
	uint32_t preasure=0;
 	short GyroXYZ[3]={0,0,0};
 	float GyroXYZ_f[3]={0,0,0};
 	short MagXYZ[3];
	short AccXYZ[3];
	
	//short AccXYZ[3];
	float AccXYZ_f[3];
	float accMag;
	short Tempreature[10];
	
	/* use LPF filter */
	//float acc_smooth[3];
		
	float ACC_Last_z=0;
	float speed_z=0;
	float pos_z=0;
	
	
	double dt=0;
	double dt_Baro=0;
	uint8_t data[2];
	uhel[0]=0;	
	uhel[1]=0;
	uhel[2]=0;
	
	uint8_t acc_i=0;
	uint8_t gyro_i=0;
	uint8_t tempr_i=0;
	uint8_t buffer[14];
	uint8_t buffer_mag[6];
	uint16_t temp[12];
	
	//taskEXIT_CRITICAL();
	MPU6050_Initialize();
// 	twi_master_options_t opt = {
// 		.speed = TWI_SPEED,
// 		.chip  = 0x1E,
// 	};
// 	
// 	twi_master_setup(TWI0, &opt);
	Mag_init();
	
 	Mag_Timer=xTimerCreate("Mag_Timer",(50/portTICK_RATE_MS),pdTRUE,0,Mag_TimerCallback);
 	if(xTimerStart(Mag_Timer,0)!=pdPASS){}
		
	/* calibrate acceleromeer*/
	KAL_ACC_XYZ calib_acc;
	//Calibrate_accel(&calib_acc);		//22
	
	calib_acc.Nasobek1=0.9981f;
	calib_acc.Nasobek2=0.9926f;
	calib_acc.X_Offset=126;
	calib_acc.Y_Offset=56;
	calib_acc.Z_Offset=-127.5f;
		
	interrupt_init();

	CurrentTime=xTaskGetTickCount();
	LastTime=xTaskGetTickCount();
	
	/* BAro_init*/
	
	//MPL3115A2_init();
//	Baro_init();
	 baro_t Baro_struct;
//	ms5611Detect(&Baro_struct);
	float baro_offset=349;
	float last_pressure=0;
	float pressure=0;
	float diff_pressure=0;
	//Baro_Calibrate(&baro_offset);
	//accAlt=baro_offset;
	//float  baroGroundAltitude = (1.0f - powf((baro_offset / 8) / 101325.0f, 0.190295f)) * 4433000.0f;
	//baro_offset=(float)(baro_offset/64);
	//baro_offset+=1;
	/*Reading Baro */
  	Baro_Timer=xTimerCreate("Timer_Baro",(20/portTICK_RATE_MS),pdTRUE,0,Baro_TimerCallback);
  	if(xTimerStart(Baro_Timer,0)!=pdPASS){}
	
		
// 	if(Kalibrace.Kalibrace==ON)
// 	{
// 		Calibrate_Comp(&MAG);
// 	}		
// 	else
// 	{
// 		MAG.Nasobek1=0.989999999;
// 		MAG.Nasobek2=1.0673854;
// 		MAG.X_Offset=-29;
// 		MAG.Y_Offset=-50;
// 		MAG.Z_Offset=51;
// 		
// 	}	
	

		
		//init Z 1G
		//EstiGravity.A[2]=acc_1G;
for (;;)
{	
	
		if(xQueueReceive(Queue_Senzor_Task,&Senzor,portMAX_DELAY)==pdPASS)
		{	
			
			if(Senzor.senzor_type==MAG_TYPE)
			{	
			 	if(Mag_get_b(buffer_mag)==0)
				{
					MAG.X=((short)((buffer_mag[1]<<8) | buffer_mag[0]));//-MAG.X_Offset);//);//+X_Offset;-30
					MAG.Y=((short)((buffer_mag[3]<<8) | buffer_mag[2]));//-MAG.Y_Offset);//);//+Y_Offset;-49
					MAG.Z=((short)((buffer_mag[5]<<8) | buffer_mag[4]));//-MAG.Z_Offset);//);-49
// 					MAG.Y*=( MAG.Nasobek1);//0.997536;
// 					MAG.Z*=( MAG.Nasobek2);//1.074;
				}

				
				 
				//Get_Filtered_Heading(uhel[0],uhel[1],&MAG,&buffer);
				
			}			
			else if(Senzor.senzor_type==BARO_TYPE)
			{	
				LastTime_Baro=CurrentTime_Baro;
				CurrentTime_Baro=xTaskGetTickCount();
				dt_Baro=(double)((CurrentTime_Baro-LastTime_Baro));
				dt_Baro/=1000;
				
				//baro_meas = (float) (((Senzor.baro[0] << 16) | (Senzor.baro[1] << 8) | (Senzor.baro[2] & 0xC0)) >> 6) + (float) ((Senzor.baro[2] & 0x30) >> 4) * 0.25;
				//baro_meas = (float) ((short) ((Senzor.baro[0] << 8) | Senzor.baro[1])) + (float) (Senzor.baro[2] >> 4) * 0.0625;
				ms5611_calculate(&tlak,&teplota);
				//baro_meas=Senzor.baro_uP
				baro_meas=(float ) applyBarometerMedianFilter((float)baro_meas);
			//	baro_meas/=100;
				// baro_meas = (float) (((Senzor.baro[0] << 16) | (Senzor.baro[1] << 8) | (Senzor.baro[2] & 0xC0)) >> 6) + (float) ((Senzor.baro[2] & 0x30) >> 4) * 0.25;
				baro_meas-=baro_offset;
				//baro_meas*=100;	//convert to mm
				
				/*mrtve pasmo*/
				if((baro_meas<(last_baro_meas+0.3f))&&(baro_meas>last_baro_meas-0.3f)) baro_meas=last_baro_meas;
// 				baro_vel = (baro_meas - last_baro_meas) * 100.0f  /dt_Baro;//
 				last_baro_meas = baro_meas;
			//	baro_vel = constrain(baro_vel, -1500, 1500);    // constrain baro velocity +/- 1500cm/s
			//	baro_vel = applyDeadband(baro_vel, 30);         // to reduce noise near zero
 			//	accAlt = (accAlt * 0.9698 + (float)(baro_meas*100) * (1.0f - 0.9698));    // complementary filter for altitude estimation (baro & acc)
 				
				// apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
				// By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
  			//	vel = (vel * 0.996 + (float)(baro_vel*100) * (1.0f - 0.996));
//  				vel_tmp = lrintf(vel);
				
				//baro_meas = filterApplyPt1(baro_meas, &Filters[BARO_MEAS], 5, dt_Baro);	//(default 17Hz
					
 			//	 Fir(&baro_meas,0);
// 				   Fir(&baro_meas,1);

			}
 			else if ((Senzor.Vycti_data==1)&&(Senzor.senzor_type==MPU_TYPE))
 			{
			//	ioport_set_pin_level(PERIODE_PIN,false);
				
				MPU9250_BufferRead(MPU6050_DEFAULT_ADDRESS,buffer,MPU6050_RA_ACCEL_XOUT_H, 14);

				AccXYZ[0]=(((short)(buffer[0]) << 8 ) | buffer[1])-calib_acc.X_Offset;
				AccXYZ[1]=(((short)(buffer[2]) << 8 ) | buffer[3])-calib_acc.Y_Offset;
				AccXYZ[2]=(((short)(buffer[4]) << 8 ) | buffer[5])-calib_acc.Z_Offset;
				
				AccXYZ[1]*=calib_acc.Nasobek1;
				AccXYZ[2]*=calib_acc.Nasobek2;
				
				GyroXYZ[0]=(((short)(buffer[8]) << 8 ) | buffer[9])-offset[0];
				GyroXYZ[1]=(((short)(buffer[10]) << 8 ) | buffer[11])-offset[1];
				GyroXYZ[2]=(((short)(buffer[12]) << 8 ) | buffer[13])-offset[2];
				
				LastTime=CurrentTime;
				CurrentTime=xTaskGetTickCount();
				dt=(double)((CurrentTime-LastTime));
				dt/=1000;
				if (dt<0.001f) dt=0.001f;
				
				
				/* Calculate .....*/
				IMU_Compute(GyroXYZ, AccXYZ,uhel,dt);
								                                                                // integrate velocity to get distance (x= a/2 * t^2)
 				accAlt = (accAlt * 0.9992 + (float)(baro_meas/10) * (1.0f - 0.9992));    // complementary filter for altitude estimation (baro & acc)
				//accAlt=filterApplyPt1((accAlt ),&Filters[FINAL_ALT],15,dt);
				last_baro_meas = (last_baro_meas * 0.999 + (float)(accAlt) * (1.0f - 0.999));
				
 				///vel = speed_z;
				//altitude_akce=accAlt;
// 				ACC_Last_z=(float)(AccXYZ[2]*0.00012207f);	//pro +-4G = acc*8/65535
 				//speed_z+=(float)(ACC_Last_z*dt);
// 				pos_z+=(float)(speed_z*dt);
							
 			}else if(Senzor.senzor_type==GPS_TYPE)
 			{
	 			//temp_gps_alt= (float) ((short) ((Senzor.gps_alt[0] << 16) | Senzor.gps_alt[1]<< 8 | Senzor.gps_alt[0]));
				//temp_gps_alt/=5.1f;
				temp_gps_alt=Senzor.gps_alt;
	    	}

			/* Send new position to motor task */
			Position.pitch=(float)MAG.X;//M_PI*uhel[0]+2.7;//vel;//sqrtf(AccXYZ[0] * AccXYZ[0]+ AccXYZ[1]* AccXYZ[1]+ AccXYZ[2]* AccXYZ[2]);//AccXYZ[2] ;//accAlt;//(uhel[0]*180/M_PI);	//+1 je korekce køivì nalepeného mpu+1.7
			Position.roll=(float)MAG.Y;//180/M_PI*uhel[1]+1.7;//1000*accAlt;//accAlt;//baro_vel;//accSum[2];//100*baro_meas;//vel;//((uhel[2]));	////+1 je korekce køivì nalepeného mpu+1.6
			Position.yaw=(float)MAG.Z;//0;//-GyroXYZ[2];//baro_meas;//baro_meas;//vel;//accSum[2];//-GyroXYZ[2];
			Position.Gyro_d[0]=GyroXYZ[0];
			Position.Gyro_d[1]=GyroXYZ[1];//AccXYZ[2];
			Position.Gyro_d[2]=0;//GyroXYZ[2];
			Position.Baro=0;//accAlt;//pos_z; // vyska je v metrech
	
			Position.type_of_data=FROM_SENZOR;
			if(xQueueSend(Queue_Motor_Task,&Position,1))	//pdPASS=1-
			{
		
			}
	
 		}
				
#if (RX_NEW_CMD==1)

#elif (TX_TO_MATLAB==1)
			
// 			temp[0]=(short)(uhel[0]);//GyroXYZ[0];
// 			temp[1]=(short)0;//(uhel[1]);//GyroXYZ[1];
// 			temp[2]=(short)0;//(uhel[2]);//GyroXYZ[2];//
// 			
// 			Semtech.Buffer[0]=(uint8_t)baro_meas;	//LOW
// 			Semtech.Buffer[1]=(uint8_t)(baro_meas>>8);		//HIGH
// 			Semtech.Buffer[2]=(uint8_t) (baro_meas>>16);
// 			Semtech.Buffer[3]=(uint8_t)0;// (temp[1]>>8);
// 			Semtech.Buffer[4]=(uint8_t) 0;
// 			Semtech.Buffer[5]=(uint8_t)( 0);
// 			Semtech.Buffer[6]=(uint8_t) temp[3];
// 			Semtech.Buffer[7]=(uint8_t)( temp[3]>>8);
// 			
// 			Semtech.Stat.Data_State=RFLR_STATE_TX_INIT;
// 			Semtech.Stat.Cmd=STAY_IN_STATE;
// 			/* Send data to Matlab */
//  			if(xQueueSend(Queue_RF_Task,&Semtech,1))	//pdPASS=1-
//  			{
//  			
//  			}
#else
	# error "TX or RX?"
#endif
			
		
//   			for (uint8_t i=0;i<8;i++)
//   			{
//   			
// 			  (uart_write(UART1,Semtech.Buffer[i]));
//  	
//   			}
 			 
		
			
			//vTaskDelay(1/portTICK_RATE_MS);
		
	}
}

