/*
 * Compass_Task.h
 *
 * Created: 27.1.2014 21:45:25
 *  Author: JR
 */ 


#ifndef SENZOR_TASK_H_
#define SENZOR_TASK_H_

void Senzor_Task(void *pvParameters);
void Mag_init(void);
//void MPU_TimerCallback(xTimerHandle pxTimer);
void MPU9150_INT(void);

#define TWI_SPEED				400000//400 khz max


#define ROLL     0
#define PITCH    1
#define YAW      2
#define THROTTLE 3
#define AUX1     4
#define AUX2     5
#define AUX3     6
#define AUX4     7

#define XAXIS    0
#define YAXIS    1
#define ZAXIS    2

typedef  struct {
	float roll;
	float pitch;
	float yaw;
} EulerAngles;

//extern sensors_t sensors;

#endif /* COMPASS_TASK_H_ */