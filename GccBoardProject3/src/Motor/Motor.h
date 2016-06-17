/*
 * Motor.h
 *
 * Created: 24.02.2016 19:45:09
 *  Author: J
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_

#include "main.h"


typedef enum{
	WANTED_YAW,
	WANTED_PITCH,
	WANTED_ROLL,
	POSITION_YAW,
	POSITION_PITCH,
	POSITION_ROLL,
	ACC_SMOTH_Z,
	ACC_X,
	ACC_Y,
	ACC_Z,
	BARO_MEAS,
	FINAL_ALT,	
	FILTER_13,
	FILTER_14,
	FILTER_15,
	FILTER_16,
	FILTER_17
	
}Filters_ID;
/* struct PID */
typedef struct{
	
	float pid_sum_p;
	float pid_sum_r;
	float pid_sum_y;
	float pid_sum_alt;
	
}PID_terms;


typedef struct filterStatePt1_s {
	float state;
	float RC;
	float constdT;
} filterStatePt1_t;

extern volatile filterStatePt1_t Filters[15];

void Motor_Task(void *pvParameters);
void PWM_init(void);
void PID_Calculate(Motor_Queue *position,PID_terms *pid);
void Filter_D(float D_p,float  D_r,float D_y, float *sum);
void Motor_Update(Motor_Queue *position,PID_terms *pid);
int constrain(int amt, int low, int high);
float constrainf(float amt, float low, float high);

float filterApplyPt1(float input, filterStatePt1_t *filter, uint8_t f_cut, float dt);
float klouzal(float data);

#define PWM_FREQUENCY      480	//480
/** Period value of PWM output waveform */
#define PERIOD_VALUE       1000		//udava procentualni stridu pro 1000 => 1000=100% duty
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    0


#endif /* MOTOR_H_ */