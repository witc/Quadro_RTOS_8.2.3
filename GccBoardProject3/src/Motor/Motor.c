/*
 * Motor.c
 *
 * Created: 24.02.2016 19:44:05
 *  Author: J
 */ 

#include <asf.h>
#include <math.h>
#include <arm_math.h>
#include "FreeRTOS_V_8_Header.h"

#include "main.h"
#include "Motor.h"

#define PID_TUNE	0
#define P_PITCH		5.2f//2,60//7.60//7.0//6,6
#define I_PITCH		.5f
#define D_PITCH		0.08f//0,06//0.1//0.09//0,07

#define P_ROLL		4.5f
#define I_ROLL		0.5f
#define D_ROLL		0.06f

#define P_YAW		0.9f//.5f
#define I_YAW		0.01//0.02f
#define D_YAW		0.01//0.04f

extern	volatile	xQueueHandle		Queue_RF_Task;
extern volatile	xQueueHandle		Queue_Motor_Task;
volatile short temp=0;

pwm_channel_t pwm_channel_0;
pwm_channel_t pwm_channel_1;
pwm_channel_t pwm_channel_2;
pwm_channel_t pwm_channel_3;

float	wanted_pitch=0;
float   wanted_roll=0;
float	wanted_yaw=0;
float	wanted_power=0;
static filterStatePt1_t Filters[10], DTermState[3];

int constrain(int amt, int low, int high)
{
	if (amt < low)
	return low;
	else if (amt > high)
	return high;
	else
	return amt;
}

float constrainf(float amt, float low, float high)
{
	if (amt < low)
	return low;
	else if (amt > high)
	return high;
	else
	return amt;
}

// PT1 Low Pass filter (when no dT specified it will be calculated from the cycleTime)
float filterApplyPt1(float input, filterStatePt1_t *filter, uint8_t f_cut, float dT) {

	// Pre calculate and store RC
	if (!filter->RC) {
		filter->RC = 1.0f / ( 2.0f * (float)M_PI * f_cut );
	}

	filter->state = filter->state + dT / (filter->RC + dT) * (input - filter->state);

	return filter->state;
}


/* PID */
void PID_Calculate(Motor_Queue *position,PID_terms *pid)
{	
	
	RF_Queue Semtech;
	int16_t posli[4];
	
	volatile float p_p, p_r, p_y,p_alt;
	static float integral_p=0, integral_r=0, integral_y=0,integral_alt=0;
	float derivative_p, derivative_r, derivative_y,derivatee_alt;
	static float prev_error_p=0, prev_error_r=0, prev_error_y=0,prev_error_alt=0;
	static float  dt=0.001f;
	static portTickType CurrentTime=0;
	static portTickType LastTime=0;
	static float sum[3];
	float delta, deltaSum[3];
	static float delta1[3], delta2[3];
	static unsigned int tune=0;
	
	ioport_set_pin_level(PERIODE_PIN,true);
	LastTime=CurrentTime;
	CurrentTime=xTaskGetTickCount();
	dt=(float)((CurrentTime-LastTime));
	dt=(float)dt*0.001f;			
	if(dt<0.001)	dt=0.001;
	/* P složka */
	
	position->yaw/=2;
   	position->yaw = filterApplyPt1(position->yaw, &Filters[0], 20, dt);	//(default 17Hz
	//position->Baro=filterApplyPt1(position->Baro,&Filters[1],10,dt);
	  
	wanted_yaw = filterApplyPt1(wanted_yaw, &Filters[2], 15, dt);	//(default 17Hz
	wanted_roll = filterApplyPt1(wanted_roll, &Filters[3], 50, dt);	//(default 17Hz
	wanted_pitch = filterApplyPt1(wanted_pitch, &Filters[4], 50, dt);	//(default 17Hz
	wanted_power = filterApplyPt1(wanted_power, &Filters[5], 50, dt);	//(default 17Hz
	//position->yaw = filterApplyPt1(wanted_yaw, &Filters[3], 2, dt);	//(default 17Hz
   //	wanted_power = filterApplyPt1(wanted_power, &PTermState[1], 2, dt);	//(default 17Hz
	p_p =(float) (wanted_pitch - position->pitch);
	p_r =(float) (wanted_roll - position->roll)-12;//
	p_y =(float) (wanted_yaw - position->yaw);
	p_alt=(float) (wanted_power - position->Baro);
//   	p_r = filterApplyPt1(p_r, &PTermState[1], 20, dt);	//(default 17Hz
//   	p_y = filterApplyPt1(p_y, &PTermState[2], 20, dt);	//(default 17Hz
// 	
	/* I položka */
	integral_p = integral_p + p_p*dt;
	integral_r = integral_r + p_r*dt;
	integral_y = integral_y + p_y*dt;
				
	/* D složka */
	derivative_p =	(float) -position->Gyro_d[0];//((p_p - prev_error_p)/dt);
	derivative_r = (float)-position->Gyro_d[1];//((p_p - prev_error_p)/dt);//(float)position->Gyro_d[1];//((p_r - prev_error_r)/dt);
	derivative_y = (float)((p_y - prev_error_y)/dt);//-position->Gyro_d[2];//position->Gyro_d[2];//;
	
	prev_error_p=p_p;
	prev_error_r=p_r;
	prev_error_y=p_y;
		
	// Dterm low pass
//   	derivative_p = filterApplyPt1(derivative_p, &DTermState[0], 10, dt);
//   	derivative_r = filterApplyPt1(derivative_r, &DTermState[1], 10, dt);
//    	derivative_y = filterApplyPt1(derivative_y, &DTermState[2], 10, dt);
// 		
	integral_p=constrainf((float)(integral_p),-10,10);
	integral_r=constrainf(integral_r,-10,10);
	integral_y=constrainf(integral_y,-10,10);
	
//  	derivative_p=constrainf((float)(derivative_p), -300.0f, 300.0f);
//   	derivative_r=constrainf(derivative_r, -300.0f, 300.0f);
//   	derivative_y=constrainf(derivative_y, -300.0f, 300.0f);
		
	
	pid->pid_sum_p=constrainf((P_PITCH*p_p+I_PITCH*integral_p+D_PITCH*derivative_p),-600,600);
	pid->pid_sum_r=constrainf((P_ROLL*p_r+I_ROLL*integral_r+D_ROLL*derivative_r),-600,600);
	pid->pid_sum_y=constrainf((P_YAW*p_y+I_YAW*integral_y+D_YAW*derivative_y),-600,600);
	//pid->pid_sum_p=filterApplyPt1(pid->pid_sum_p, &DTermState[0], 17, dt);
	ioport_set_pin_level(PERIODE_PIN,false);
	
	//posli[1]=(int16_t)derivative_p;
//#if (PID_TUNE==1)

	/* posli do matlabu */
//  	posli[0]=(int16_t)position->Baro;//(P_PITCH*p_p);//+pid->I_p+pid->D_p
//  	posli[1]=(int16_t)0;//;(I_PITCH*integral_p);
//  	posli[2]=(int16_t)0;//;(D_PITCH*derivative_p);
//  	posli[3]=(int16_t)0;//;position->pitch;
//  	
//  	Semtech.Buffer[0]=(uint8_t)posli[0];	//LOW
//  	Semtech.Buffer[1]=(uint8_t)(posli[0]>>8);		//HIGH
//  	Semtech.Buffer[2]=(uint8_t) posli[1];
//  	Semtech.Buffer[3]=(uint8_t) (posli[1]>>8);
//  	Semtech.Buffer[4]=(uint8_t) posli[2];
//  	Semtech.Buffer[5]=(uint8_t)( posli[2]>>8);
//  	Semtech.Buffer[6]=(uint8_t) posli[3];
//  	Semtech.Buffer[7]=(uint8_t)( posli[3]>>8);
//  	
//  	Semtech.Stat.Data_State=RFLR_STATE_TX_INIT;
//  	Semtech.Stat.Cmd=STAY_IN_STATE;
	/* Send data to Matlab */

//    	for (uint8_t i=0;i<2;i++)
//    	{
//    		usart_putchar((Usart*)UART1,Semtech.Buffer[0+i]);
//     	}
//  	if(xQueueSend(Queue_RF_Task,&Semtech,1)==pdPASS)	//pdPASS=1-
//  	{
//  		
//  	}

	
//#endif

}


void Motor_Update(Motor_Queue *position,PID_terms *pid)
{	
	int16_t Power_M1=0;
	int16_t Power_M2=0;
	int16_t Power_M3=0;
	int16_t Power_M4=0;
	
	int16_t Final_M1=0;
	int16_t Final_M2=0;
	int16_t Final_M3=0;
	int16_t Final_M4=0;
	
	/* PITCH */
    Power_M1+=(int16_t)(pid->pid_sum_r-pid->pid_sum_p-pid->pid_sum_y);//++pid->I_p+pid->D_p)
    Power_M3+=(int16_t)(-pid->pid_sum_r-pid->pid_sum_p+pid->pid_sum_y);
    Power_M2+=(int16_t)(pid->pid_sum_r+pid->pid_sum_p+pid->pid_sum_y);
    Power_M4+=(int16_t)(-pid->pid_sum_r+pid->pid_sum_p-pid->pid_sum_y);
// 	
// 	
// 	/* ROLL */
//    	Power_M1+=+(int16_t)(pid->pid_sum_r);//);//+pid->I_p+pid->D_p;
//    	Power_M2+=Power_M1;
//    	Power_M3+=-Power_M1;
   	//Power_M4+=-Power_M1;
// 	 
// 	 /*YAW */
//  	 Power_M1+=+(int16_t)(pid->pid_sum_y);//);//+pid->I_p+pid->D_p;+I_ROLL*pid->I_r+D_ROLL*pid->D_p
//  	 Power_M4+=Power_M1;
//  	 Power_M3+=-Power_M1;
//  	 Power_M2+=-Power_M1;
	 
#if (PID_TUNE==1)
	wanted_power=130;
#endif	
	
	/* offset na plynové páèce - mrtvá zona */
	if (wanted_power>10)
	{	
// 		if (wanted_power>Power_M1)Final_M1=(int16_t)(wanted_power+Power_M1);
// 		if (wanted_power>Power_M2)Final_M2=(int16_t)(wanted_power+Power_M2);
// 		if (wanted_power>Power_M3)Final_M3=(int16_t)(wanted_power+Power_M3);
// 		if (wanted_power>Power_M4)Final_M4=(int16_t)(wanted_power+Power_M4);
		
		Final_M1=(int16_t)(wanted_power+Power_M1);
 		Final_M2=(int16_t)(wanted_power+Power_M2);
 		Final_M3=(int16_t)(wanted_power+Power_M3);
 		Final_M4=(int16_t)(wanted_power+Power_M4);
	
	}
	else
	{	
		/* motory stojí */
		Final_M1=10;
		Final_M2=10;
		Final_M3=10;
		Final_M4=10;
	}
	
	/* maximum hodnot pro PWM jw 900 - plný výkon*/
// 	if (Final_M1>900) Final_M1=900;
// 	if (Final_M2>900) Final_M2=900;
// 	if (Final_M3>900) Final_M3=900;
// 	if (Final_M4>900) Final_M4=900;
	
	Final_M1=constrain(Final_M1,10,900);
	Final_M2=constrain(Final_M2,10,900);
	Final_M3=constrain(Final_M3,10,900);
	Final_M4=constrain(Final_M4,10,900);
	
	pwm_channel_disable(PWM, 0);
	pwm_channel_disable(PWM, 1);
	pwm_channel_disable(PWM, 2);
	pwm_channel_disable(PWM, 3);
	
	pwm_channel_update_duty(PWM,&pwm_channel_0,Final_M1);
	pwm_channel_update_duty(PWM,&pwm_channel_1,Final_M2);
	pwm_channel_update_duty(PWM,&pwm_channel_2,Final_M3);
	pwm_channel_update_duty(PWM,&pwm_channel_3,Final_M4);
	
	pwm_channel_enable(PWM, 0);
	pwm_channel_enable(PWM, 1);
	pwm_channel_enable(PWM, 2);
	pwm_channel_enable(PWM, 3);
	

		
}
/* PWM init function */
void PWM_init(void)
{	
	pmc_enable_periph_clk(ID_PWM);
	pmc_enable_periph_clk(ID_PIOA);
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOC);
	
	/* pins */
	pio_configure_pin(PWM_M1,PIO_TYPE_PIO_PERIPH_B);
	ioport_disable_pin(PWM_M1);	
	ioport_set_pin_dir(PWM_M1,IOPORT_DIR_OUTPUT);
	
	pio_configure_pin(PWM_M2,PIO_TYPE_PIO_PERIPH_A);
	ioport_disable_pin(PWM_M2);
	ioport_set_pin_dir(PWM_M2,IOPORT_DIR_OUTPUT);
	
	pio_configure_pin(PWM_M3,PIO_TYPE_PIO_PERIPH_B);
	ioport_disable_pin(PWM_M3);
	ioport_set_pin_dir(PWM_M3,IOPORT_DIR_OUTPUT);
	
	pio_configure_pin(PWM_M4,PIO_TYPE_PIO_PERIPH_B);
	ioport_disable_pin(PWM_M4);
	ioport_set_pin_dir(PWM_M4,IOPORT_DIR_OUTPUT);
	 
	/* Disable PWM channels*/
	pwm_channel_disable(PWM, 0);
	pwm_channel_disable(PWM, 1);
	pwm_channel_disable(PWM, 2);
	pwm_channel_disable(PWM, 3);
		
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_peripheral_bus_hz(PWM)
	};
	pwm_init(PWM, &clock_setting);
	
	/* Period is left-aligned */
	pwm_channel_0.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a low level */
	pwm_channel_0.polarity =PWM_HIGH;
	/* Use PWM clock A as source clock */
	pwm_channel_0.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	pwm_channel_0.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	pwm_channel_0.ul_duty = 900;
	pwm_channel_0.channel = 0;
	pwm_channel_init(PWM, &pwm_channel_0);
	
	/* Period is left-aligned */
	pwm_channel_1.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a low level */
	pwm_channel_1.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	pwm_channel_1.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	pwm_channel_1.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	pwm_channel_1.ul_duty = 900;
	pwm_channel_1.channel = 1;
	pwm_channel_init(PWM, &pwm_channel_1);
	
	/* Period is left-aligned */
	pwm_channel_2.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a low level */
	pwm_channel_2.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	pwm_channel_2.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	pwm_channel_2.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	pwm_channel_2.ul_duty = 900;
	pwm_channel_2.channel = 2;
	pwm_channel_init(PWM, &pwm_channel_2);
	
	/* Period is left-aligned */
	pwm_channel_3.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a low level */
	pwm_channel_3.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	pwm_channel_3.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	pwm_channel_3.ul_period = PERIOD_VALUE;
	/* Duty cycle value of output waveform */
	pwm_channel_3.ul_duty = 900;
	pwm_channel_3.channel = 3;
	pwm_channel_init(PWM, &pwm_channel_3);

	/*dissable int */
	pwm_channel_disable_interrupt(PWM, 0, 0);
	pwm_channel_disable_interrupt(PWM, 1, 0);
	pwm_channel_disable_interrupt(PWM, 2, 0);
	pwm_channel_disable_interrupt(PWM, 3, 0);
	
	pwm_channel_0.channel = PWM_CHANNEL_0;
	pwm_channel_1.channel = PWM_CHANNEL_1;
	pwm_channel_2.channel = PWM_CHANNEL_2;
	pwm_channel_3.channel = PWM_CHANNEL_3;
	pwm_channel_update_duty(PWM,&pwm_channel_0,900);
	pwm_channel_update_duty(PWM,&pwm_channel_1,900);
	pwm_channel_update_duty(PWM,&pwm_channel_2,900);
	pwm_channel_update_duty(PWM,&pwm_channel_3,900);
 	
 	/* Enable PWM channels */
 	pwm_channel_enable(PWM, 0);
 	pwm_channel_enable(PWM, 1);
 	pwm_channel_enable(PWM, 2);
 	pwm_channel_enable(PWM, 3);
  	vTaskDelay(1000/portTICK_RATE_MS);
	
	pwm_channel_disable(PWM, 0);
	pwm_channel_disable(PWM, 1);
	pwm_channel_disable(PWM, 2);
	pwm_channel_disable(PWM, 3);
	
	pwm_channel_update_duty(PWM,&pwm_channel_0,10);
	pwm_channel_update_duty(PWM,&pwm_channel_1,10);
	pwm_channel_update_duty(PWM,&pwm_channel_2,10);
	pwm_channel_update_duty(PWM,&pwm_channel_3,10);
	/* Enable PWM channels */
	pwm_channel_enable(PWM, 0);
	pwm_channel_enable(PWM, 1);
	pwm_channel_enable(PWM, 2);
	pwm_channel_enable(PWM, 3);
	vTaskDelay(500/portTICK_RATE_MS);

	
}
 
 
 void Motor_Task(void *pvParameters)
 {
	Motor_Queue Position;
	PID_terms	Pid;
	uint16_t y0_output=0;
	/*expo*/
	uint8_t		k=2;
	uint8_t first_RX=1;
	int16_t offset[4];
	/* PWM inicializace */ 
 	PWM_init();
	
	
	while(1)
	{
 		if(xQueueReceive(Queue_Motor_Task,&Position,portMAX_DELAY)==pdPASS)
 		{	
 			if (Position.type_of_data==FROM_SENZOR)
 			{
 				PID_Calculate(&Position,&Pid);
				Motor_Update(&Position,&Pid);
			

 			}
 			else if(Position.type_of_data==FROM_TX)
 			{	
				 
				 
				if (first_RX==1)
				{	
					offset[0]=2500-Position.TX_CH_xx[0];
					offset[1]=2500-Position.TX_CH_xx[1];
					offset[2]=2500-Position.TX_CH_xx[2];
					offset[3]=3010-Position.TX_CH_xx[3];
					
					first_RX=0;
				}
					
					Position.TX_CH_xx[0]+=offset[0];
					Position.TX_CH_xx[1]+=offset[1];
					Position.TX_CH_xx[2]+=offset[2];
					Position.TX_CH_xx[3]+=offset[3];
					
					wanted_power=(float)(Position.TX_CH_xx[3]*(-0.001f)+3.0f);
					wanted_power=(float)(2.702f*wanted_power*wanted_power*wanted_power-3.7582f*wanted_power*wanted_power+2.096f*wanted_power-0.0542f);
					wanted_power=(float)((wanted_power*900));
					if (wanted_power>900) wanted_power=900;
					if (wanted_power<1) wanted_power=1;
					
					
					 
					 /* Pøevod na +- 1*/
					 wanted_pitch=(float)(Position.TX_CH_xx[2]*(-0.002f)+5);
					 wanted_roll=(float)((Position.TX_CH_xx[0]*(-0.002f)+5));	
					 wanted_yaw=(float)(Position.TX_CH_xx[1]*(-0.002f)+5);
					 /*pøevod na expo +-1  z +-1 */
					 wanted_pitch=(float)((wanted_pitch*wanted_pitch*wanted_pitch*(k-1)+wanted_pitch)/k);
					 wanted_roll=(float)((wanted_roll*wanted_roll*wanted_roll*(k-1)+wanted_roll)/k);
					// wanted_yaw=(float)((wanted_yaw*wanted_yaw*wanted_yaw*(1-1)+wanted_yaw)/1);
					 /* pøevod na +-45 z +-1*/
					 wanted_pitch=(float)(wanted_pitch*45);
					 wanted_roll=(float)((wanted_roll*45));//odeèitam konstatni 4 stupne kvuli baterce - zatez
					 
					 wanted_pitch=constrainf(wanted_pitch,-45,45);
					 wanted_roll=constrainf(wanted_roll,-45,45);
					 
					 /* pøevod na w rychlost +- 1000 z expo */
					 wanted_yaw=(float)(wanted_yaw*1000);
				
		
 			}
 			
 			
 			
 		}
			 	
	}
 }