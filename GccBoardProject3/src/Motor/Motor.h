/*
 * Motor.h
 *
 * Created: 24.02.2016 19:45:09
 *  Author: J
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_

//#include "main.h"

/* struct PID */
typedef struct{
	
	float pid_sum_p;
	float pid_sum_r;
	float pid_sum_y;
	
}PID_terms;


typedef struct filterStatePt1_s {
	float state;
	float RC;
	float constdT;
} filterStatePt1_t;

void Motor_Task(void *pvParameters);
void PWM_init(void);
void PID_Calculate(Motor_Queue *position,PID_terms *pid);
void Filter_D(float D_p,float  D_r,float D_y, float *sum);
void Motor_Update(Motor_Queue *position,PID_terms *pid);
int constrain(int amt, int low, int high);
float constrainf(float amt, float low, float high);

float filterApplyPt1(float input, filterStatePt1_t *filter, uint8_t f_cut, float dt);

#define PWM_FREQUENCY      480	//480
/** Period value of PWM output waveform */
#define PERIOD_VALUE       1000		//udava procentualni stridu pro 1000 => 1000=100% duty
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    0


#endif /* MOTOR_H_ */