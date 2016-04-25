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
//volatile	MPU9150_Buffer XYZ;
	
void Gyro_Angle(short g_x,short g_y,short g_z,EulerAngles *uhly,float dt);
void Akce_Angle(short a_x, short a_y, short a_z,EulerAngles *uhly);

void Mag_TimerCallback(xTimerHandle pxTimer)
{
	Senzor_Queue Data_Queue_MAG;
	signed portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken=pdTRUE;	//pøerušení se dokonèí celé= pdFalse
	
	
	if(Mag_get_b(Data_Queue_MAG.mag)==0)
	{
		Data_Queue_MAG.senzor_type=MAG_TYPE;
		//Data_Queue_BARO.Vycti_data=1;
		xQueueSendToBackFromISR(Queue_Senzor_Task,&Data_Queue_MAG,&xHigherPriorityTaskWoken);
		//pio_enable_interrupt(PIOB, PIO_PB0);
		
		if (xHigherPriorityTaskWoken==pdTRUE) portYIELD();
		
	}
}

void Baro_TimerCallback(xTimerHandle pxTimer)
{	
	Senzor_Queue Data_Queue_BARO;
	signed portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken=pdTRUE;	//pøerušení se dokonèí celé= pdFalse
	uint8_t data=0;
	
	if(Baro_get_preasure(Data_Queue_BARO.baro)==0)
	{	
		Data_Queue_BARO.senzor_type=BARO_TYPE;
	 	//Data_Queue_BARO.Vycti_data=1;
	 	xQueueSendToBackFromISR(Queue_Senzor_Task,&Data_Queue_BARO,&xHigherPriorityTaskWoken);
	 	//pio_enable_interrupt(PIOB, PIO_PB0);
	 	
	 	if (xHigherPriorityTaskWoken==pdTRUE) portYIELD();
		 
	}
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


void Gyro_Angle(short g_x,short g_y,short g_z,EulerAngles *uhly, float dt)
{
	#define GAA			 2
	
	uint8_t			data_av=0;
	static int		Suma1=0;
	static int		Suma2=0;
	static int		Suma3=0;
	
	static short		pole1[GAA];
	static short		pole2[GAA];
	static short		pole3[GAA];
	
	static unsigned int	counter=0;
	static short		Temp1 = 0;
	static short		Temp2 = 0;
	static short		Temp3 = 0;
	
	
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


void Akce_Angle(short a_x, short a_y, short a_z,EulerAngles *uhly)
{
	#define AAA		25
		
	uint8_t				data_av=0;
	static int32_t		Suma1=0;
	static int32_t		Suma2=0;
	static int32_t		Suma3=0;
	
	static int16_t		pole1[AAA];
	static int16_t		pole2[AAA];
	static int16_t		pole3[AAA];
	
	static unsigned int	counter=0;
	double		Temp1 = 0;
	double		Temp2 = 0;
	double       Temp3 = 0;
	
	
	
	Suma1+=a_x;
	pole1[counter]=a_x;
	Suma2+=a_y;
	pole2[counter]=a_y;
	Suma3+=a_z;
	pole3[counter]=a_z;
	counter++;

	if (counter==AAA) counter=0;

	Suma1-=pole1[counter];
	Temp1=(Suma1/AAA);
	Suma2-=pole2[counter];
	Temp2=(Suma2/AAA);
	Suma3-=pole3[counter];
	Temp3=(Suma3/AAA);
	
	uhly->pitch=(float)((atan2((double)Temp2,(double)Temp3)*(180)/3.141f));
	//Alfa1-=90;
// 	if (Alfa1>90)  Alfa1=90;
// 	if (Alfa1<-90) Alfa1=-90;
	
	
	//uhly->pitch=(float)(Alfa1);
	
	uhly->roll=(float)((-atan2((double)Temp1,(double)Temp3)*(180)/3.141f));
	//Alfa2-=90;
// 	if (Alfa2>90)  Alfa2=90;
// 	if (Alfa2<-90) Alfa2=-90;
	
	//uhly->pitch=(float)(Alfa2);
		
}



void Senzor_Task(void *pvParameters)
{	
	Senzor_Queue Senzor;
	//MPU9150_Buffer XYZ;
	Motor_Queue Position;
	RF_Queue Semtech;
	
	 MAG_XYZ MAG;
 	EulerAngles Angles_A;
	portTickType CurrentTime;
	portTickType LastTime;
	
	#define CONST_FILTER 0.998f
		
	char Kalibrace=NULL;
	static float uhel[3];
	float f_temp[3];
	uint16_t packet_count=0;
	
	float baro_meas=0;
 	short GyroXYZ[30];
 	short MagXYZ[3];
	short AccXYZ[30];
	short Tempreature[10];
	
	double dt=0;
	uint8_t data[2];
	uhel[0]=0;	
	uhel[1]=0;
	uhel[2]=0;
	
	uint8_t acc_i=0;
	uint8_t gyro_i=0;
	uint8_t tempr_i=0;
	uint16_t save_count=0;
	uint8_t buffer[14];
	uint16_t temp[12];
	
	//taskEXIT_CRITICAL();
	MPU6050_Initialize();
	//taskEXIT_CRITICAL();
//	MPU9150_Gyro_Tempr_Bias_no_fifo(offset);
 	offset[0]=-5;
 	offset[1]=1;
 	offset[2]=3;
	
	interrupt_init();

	CurrentTime=xTaskGetTickCount();
	LastTime=xTaskGetTickCount();
	
	/* BAro_init*/
	/*reset */
// 	uint8_t tmp= MPL3115A2_CTRL_REG1_RST;
// 	Baro_send(MPL3115A2_CTRL_REG1,&tmp,1);
// 	vTaskDelay(10/portTICK_RATE_MS);
		
//	Baro_init();
	float baro_offset=0;
	float last_pressure=0;
	float pressure=0;
	float diff_pressure=0;
	//Baro_Calibrate(&baro_offset);
	//Baro_init();
	
	/*Reading Baro */
//  	Baro_Timer=xTimerCreate("Timer_Baro",(10/portTICK_RATE_MS),pdTRUE,0,Baro_TimerCallback);
//  	if(xTimerStart(Baro_Timer,0)!=pdPASS){}
	
	//Mag_init();	
		
// 	if(Kalibrace.Kalibrace==ON)
// 	{
 		//Calibrate_Comp(&MAG);
// 	}		
		
	MAG.Nasobek1=0.989999999;
	MAG.Nasobek2=1.0673854;
	MAG.X_Offset=-29;
	MAG.Y_Offset=-50;
	MAG.Z_Offset=51;
		 
	Mag_Timer=xTimerCreate("Mag_Timer",(20/portTICK_RATE_MS),pdTRUE,0,Mag_TimerCallback);
	if(xTimerStart(Mag_Timer,0)!=pdPASS){}
		
		
for (;;)
{	
	
		if(xQueueReceive(Queue_Senzor_Task,&Senzor,portMAX_DELAY)==pdPASS)
		{	
			
			if(Senzor.senzor_type==MAG_TYPE)
			{	
				MAG.X=((short)((Senzor.mag[0]<<8) | Senzor.mag[1])-MAG.X_Offset);//);//+X_Offset;-30
				MAG.Y=((short)((Senzor.mag[2]<<8) | Senzor.mag[3])-MAG.Y_Offset);//);//+Y_Offset;-49
				MAG.Z=((short)((Senzor.mag[4]<<8) | Senzor.mag[5])-MAG.Z_Offset);//);-49
				MAG.Y*=( MAG.Nasobek1);//0.997536;
				MAG.Z*=( MAG.Nasobek2);//1.074;
				 
				Get_Filtered_Heading(uhel[0],uhel[1],&MAG,&buffer);
				
			}			
			else if(Senzor.senzor_type==BARO_TYPE)
			{
				 /* Get altitude, the 20-bit measurement in meters is comprised of a signed integer component and
				   a fractional component. The signed 16-bit integer component is located in OUT_P_MSB and OUT_P_CSB.
				   The fraction component is located in bits 7-4 of OUT_P_LSB. Bits 3-0 of OUT_P_LSB are not used */
				   //baro_meas = (float) ((short) (((Senzor.baro[0] << 8)) | Senzor.baro[1])) + (float) (Senzor.baro[2] >> 4) * 0.0625;
				 
			   
			   last_pressure = pressure;
			   pressure = (float)(Senzor.baro[2])*0.005 + 0.995*pressure;
			   
			   diff_pressure = last_pressure - pressure;
			   
				   
				  // baro_meas=44330*(1-pow())
					//baro_meas=baro_meas-baro_offset;
					///* Get pressure, the 20-bit measurement in Pascals is comprised of an unsigned integer component and a fractional component.
					//The unsigned 18-bit integer component is located in OUT_P_MSB, OUT_P_CSB and bits 7-6 of OUT_P_LSB.
					//The fractional component is located in bits 5-4 of OUT_P_LSB. Bits 3-0 of OUT_P_LSB are not used.*/
					 //
				//	baro_meas = (float) (((Senzor.baro[2] << 16) | (Senzor.baro[1] << 8) | (Senzor.baro[0] & 0xC0)) >> 6) + (float) ((Senzor.baro[0] & 0x30) >> 4) * 0.25;
// 					uint32_t tlak=0;
// 					tlak=(uint16_t)baro_meas;
// 					Semtech.Buffer[0]=(uint8_t)tlak;	//LOW
// 					Semtech.Buffer[1]=(uint8_t)(tlak>>8);		//HIGH
// 					Semtech.Buffer[2]=(uint8_t) (tlak>>16);
// 					Semtech.Buffer[3]=(uint8_t)0;// (temp[1]>>8);
// 					Semtech.Buffer[4]=(uint8_t) 0;
// 					Semtech.Buffer[5]=(uint8_t)( 0);
// 					Semtech.Buffer[6]=(uint8_t) temp[3];
// 					Semtech.Buffer[7]=(uint8_t)( temp[3]>>8);
// 					
// 					Semtech.Stat.Data_State=RFLR_STATE_TX_INIT;
// 					Semtech.Stat.Cmd=STAY_IN_STATE;
// 					/* Send data to Matlab */
// 					if(xQueueSend(Queue_RF_Task,&Semtech,1))	//pdPASS=1-
// 					{
// 						
// 					}
			}
 			else if ((Senzor.Vycti_data==1)&&(Senzor.senzor_type==MPU_TYPE))
 			{
			//	ioport_set_pin_level(PERIODE_PIN,false);
				
				MPU9250_BufferRead(MPU6050_DEFAULT_ADDRESS,buffer,MPU6050_RA_ACCEL_XOUT_H, 14);

				AccXYZ[0]=(((short)(buffer[0]) << 8 ) | buffer[1]);
				AccXYZ[1]=(((short)(buffer[2]) << 8 ) | buffer[3]);
				AccXYZ[2]=(((short)(buffer[4]) << 8 ) | buffer[5]);
			
				GyroXYZ[0]=(((short)(buffer[8]) << 8 ) | buffer[9])-offset[0];
				GyroXYZ[1]=(((short)(buffer[10]) << 8 ) | buffer[11])-offset[1];
				GyroXYZ[2]=(((short)(buffer[12]) << 8 ) | buffer[13])-offset[2];
				
// 				AccXYZ[0]=(((short)(buffer[0]) << 8 ) | buffer[1]);
// 				AccXYZ[1]=(((short)(buffer[2]) << 8 ) | buffer[3]);
// 				AccXYZ[2]=(((short)(buffer[4]) << 8 ) | buffer[5]);
// 				
// 				GyroXYZ[0]=(((short)(buffer[8]) << 8 ) | buffer[9])-offset[0];
// 				GyroXYZ[1]=(((short)(buffer[10]) << 8 ) | buffer[11])-offset[1];
// 				GyroXYZ[2]=(((short)(buffer[12]) << 8 ) | buffer[13])-offset[2];
			
			
				Akce_Angle(AccXYZ[0],AccXYZ[1],AccXYZ[2],&Angles_A);
				LastTime=CurrentTime;
				CurrentTime=xTaskGetTickCount();
				dt=(double)((CurrentTime-LastTime));
				dt/=1000;
				//uhel[0] =(uhel[0]+(GyroXYZ[0]*0.06103515f*dt)) ;
				
 				uhel[0] =CONST_FILTER*(uhel[0]+(float)(GyroXYZ[0]*0.06103515f*dt)) + (1-CONST_FILTER)*Angles_A.pitch;
 				uhel[1] =CONST_FILTER*(uhel[1]+(float)(GyroXYZ[1]*0.06103515f*dt)) + (1-CONST_FILTER)*Angles_A.roll;
 				//uhel[2] +=(float)(GyroXYZ[2]*0.06103515f*dt);
				
				//
				//uhel[2]=(float)(4*AccXYZ[0]/65535);
				//uhel[0]=uhel[0]+(AccXYZ[0]*dt);
				//uhel[1]=(float)AccXYZ[0];//uhel[1]+(uhel[0]*dt);
			
 			}
// 			
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
 			 
			/* Send new position to motor task */
			Position.pitch=(float)uhel[0];
			Position.roll=(float)uhel[1];
			Position.yaw=(float)-GyroXYZ[2];
			Position.Gyro_d[0]=GyroXYZ[0];
			Position.Gyro_d[1]=GyroXYZ[1];
			Position.Gyro_d[2]=GyroXYZ[2];
			Position.Baro=baro_meas;
				
 			Position.type_of_data=FROM_SENZOR;		
 			if(xQueueSend(Queue_Motor_Task,&Position,1))	//pdPASS=1-
 			{
 				 
 			}
			
			//vTaskDelay(1/portTICK_RATE_MS);
		
	}
}

