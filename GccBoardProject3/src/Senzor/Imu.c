///*
 //* Imu.c
 //*
 //* Created: 06.05.2016 10:55:39
 //*  Author: J
 //*/ 
//
#include <math.h>
#include <asf.h>
#include "stdlib.h"
#include "Imu.h"
#include "Motor.h"
#include "FreeRTOS_V_8_Header.h"

/* scale for get deg/s from Gyro*/
float scale=0.061035f;
float deltaGyroAngle[3];

/* RAW value for 1 G */
short acc_1G=8178;
int32_t accSum[3];

/*Gravity vector*/
fp_vect EstiGravity;

float  accAlt=0,vel=0;

#define FIR_HP 22
const float FIR_HPass[22]=  {
	-0.002747379929816, 0.003638146822209, 0.001290138590684, -0.01280559110698,
	0.02090462382392,-0.008970485586287, -0.02951470247043,  0.07349565726954,
	-0.07288680587089, -0.04333934189475,   0.5701777796257,   0.5701777796257,
	-0.04333934189475, -0.07288680587089,  0.07349565726954, -0.02951470247043,
	-0.008970485586287,  0.02090462382392, -0.01280559110698, 0.001290138590684,
	0.003638146822209,-0.002747379929816
};


void IMU_Compute(short *GyroAngle, short *Accel,float *uhel, float dt)
{	
	float accZ_tmp,vel_acc;
	
	float AccMagnitude=0;
	for (uint8_t axis=0;axis<3;axis++)
	{
		deltaGyroAngle[axis] = (float) GyroAngle[axis] * scale*dt*M_PI/180;
		AccMagnitude += (float)Accel[axis] * Accel[axis];
	}
	AccMagnitude = (float) (AccMagnitude / (acc_1G*acc_1G)); //should be  equal to 1 =1G
		
	/* Do a rotate vect*/
	rotate_DCM(&EstiGravity, deltaGyroAngle);
	
	/* complementar filter  - compensate gyro drift*/
	/*if whole acceleration is useful - repair the mistake of computed  gravity*/
	if ((AccMagnitude>0.7)&&(AccMagnitude<1.33))
 	{
		for (uint8_t axis = 0; axis < 3; axis++)
		{
			//EstiGravity.A[axis] = (EstiGravity.A[axis] * 600 + AccXYZ[axis]) *  INV_GYR_CMPF_FACTOR;
			EstiGravity.XYZ[axis] = (float) (EstiGravity.XYZ[axis] *0.9985f)+(Accel[axis]*(float)(1-0.9985f));
		}
	}
	
	// Attitude of the estimated vector
	uhel[0] = atan2f(EstiGravity.XYZ[1], EstiGravity.XYZ[2]);
	uhel[1]= atan2f(-EstiGravity.XYZ[0], sqrtf(EstiGravity.XYZ[1] * EstiGravity.XYZ[1] + EstiGravity.XYZ[2] * EstiGravity.XYZ[2]));

	// 				 if (sensors(SENSOR_MAG)) {
	// 					 rotateV(&EstM.V, deltaGyroAngle);
	// 					 for (axis = 0; axis < 3; axis++)
	// 					 EstM.A[axis] = (EstM.A[axis] * (float)mcfg.gyro_cmpfm_factor + magADC[axis]) * INV_GYR_CMPFM_FACTOR;
	// 					 heading = calculateHeading(&EstM);
	// 					 } else {
  //	rotate_DCM(&EstiGravity, deltaGyroAngle);
	  	//normalizeV(&EstiGravity, &EstiGravity);
//  	uhel[2] = calculateHeading(&EstiN,uhel[0],uhel[1]);
	// //
	acc_calc(Accel,uhel,dt); // rotate acc vector into earth frame
		
	vel_acc = accSum[2] *dt*0.0001220703f; // in m
	// Integrator - Altitude in cm
	accAlt += (vel_acc * 0.5f) * dt + vel * dt;                                         // integrate velocity to get distance (x= a/2 * t^2)
	
	if(vel>0)	vel-=0.00002f;
  	if(vel<0)	vel+=0.00002f;
	vel += vel_acc;
	
	//vel = applyDeadband((vel), 0.009f);
	// 				// Integrator - Altitude in cm
	//speed_z = (float)accSum[2] * dt;
	//pos_z += (accSum[2] * 0.5f) * dt + vel * dt;
	
}

/* rotate acceleration to Earth frame and calculate acceleration in it (global frame)*/
void acc_calc(short *AccXYZ,float *angle_radian, float dt)
{
	static float accZ_smooth = 0;
	float rpy[3];
	fp_vect accel_ned;
	// the accel values have to be rotated into the earth frame
	rpy[0] = -(float)angle_radian[0];
	rpy[1] = -(float)angle_radian[1];
	rpy[2] = -(float)angle_radian[2];

	accel_ned.XYZ[0] = AccXYZ[0];
	accel_ned.XYZ[1]=  AccXYZ[1];
	accel_ned.XYZ[2] = AccXYZ[2];

	rotate_DCM(&accel_ned, rpy);
	//
	// 	if (cfg.acc_unarmedcal == 1)
	// 	{
	// 		if (!f.ARMED) {
	// 			accZoffset -= accZoffset / 64;
	// 			accZoffset += accel_ned.V.Z;
	// 		}
	// 		accel_ned.V.Z -= accZoffset / 64;  // compensate for gravitation on z-axis
	// 	} else
	accel_ned.XYZ[2] -= acc_1G;

	//accz_smooth = accz_smooth + (dT / (fc_acc + dT)) * (accel_ned.V.Z - accz_smooth); // low pass filter- accz_smooth
	//accZ_smooth=filterApplyPt1((accel_ned.XYZ[2] ),&Filters[ACC_SMOTH_Z],15,dt);
	accZ_smooth=accel_ned.XYZ[2];
	// apply Deadband to reduce integration drift and vibration influence and
	// sum up Values for later integration to get velocity and distance
	accSum[0] = applyDeadband((accel_ned.XYZ[0]), 100);
	accSum[1] = applyDeadband((accel_ned.XYZ[1]), 100);
	accSum[2] = applyDeadband((accZ_smooth), 100);

	// 	accTimeSum += deltaT;
	// 	accSumCount++;
	
} 

/* Rotation matrix DCM - compute actual rotation */
void rotate_DCM( fp_vect *vect, float *delta)
{
	fp_vect v_tmp;
	
	v_tmp.XYZ[0]=vect->XYZ[0];
	v_tmp.XYZ[1]=vect->XYZ[1];
	v_tmp.XYZ[2]=vect->XYZ[2];
	
	float mat[3][3];
	float cosx, sinx, cosy, siny, cosz, sinz;
	float coszcosx, sinzcosx, coszsinx, sinzsinx;

	cosx = cosf(delta[0]);
	sinx = sinf(delta[0]);
	cosy = cosf(delta[1]);
	siny = sinf(delta[1]);
	cosz = cosf(delta[2]);
	sinz = sinf(delta[2]);
	
	/* small angle aproximation*/
	// 	cosx = 1;
	// 	sinx = (delta[0]);
	// 	cosy = 1;
	// 	siny = (delta[1]);
	// 	cosz = 1;
	// 	sinz = (delta[2]);

	coszcosx = cosz * cosx;
	sinzcosx = sinz * cosx;
	coszsinx = sinx * cosz;
	sinzsinx = sinx * sinz;

	mat[0][0] = cosz * cosy;
	mat[0][1] = -cosy * sinz;
	mat[0][2] = siny;
	mat[1][0] = sinzcosx + (coszsinx * siny);
	mat[1][1] = coszcosx - (sinzsinx * siny);
	mat[1][2] = -sinx * cosy;
	mat[2][0] = (sinzsinx) - (coszcosx * siny);
	mat[2][1] = (coszsinx) + (sinzcosx * siny);
	mat[2][2] = cosy * cosx;

	vect->XYZ[0] = v_tmp.XYZ[0] * mat[0][0] + v_tmp.XYZ[1]* mat[1][0] + v_tmp.XYZ[2] * mat[2][0];
	vect->XYZ[1] = v_tmp.XYZ[0] * mat[0][1] + v_tmp.XYZ[1] * mat[1][1] + v_tmp.XYZ[2]* mat[2][1];
	vect->XYZ[2] = v_tmp.XYZ[0] * mat[0][2] + v_tmp.XYZ[1] * mat[1][2] + v_tmp.XYZ[2] * mat[2][2];
}


float applyDeadband(float value, float deadband)
{
	if (abs(value) < deadband) {
		value = 0;
		} 
// 		else if (value > 0) {
// 		value -= deadband;
// 		} else if (value < 0) {
// 		value += deadband;
// 		}
	return value;
}


/* Normalize a vector*/
void normalizeV(struct fp_vector *src, struct fp_vector *dest)
{
	float length;

	length = sqrtf(src->XYZ[0] * src->XYZ[0]+ src->XYZ[1]* src->XYZ[1]+ src->XYZ[2]* src->XYZ[2]);
	if (length != 0) {
		dest->XYZ[0] = src->XYZ[0] / length;
		dest->XYZ[1] = src->XYZ[1] / length;
		dest->XYZ[2] = src->XYZ[2] / length;
	}
}


void HP_Fir(float *data,short id)
{
	float Temp=0;
	static float Temp_pole[6][FIR_HP];
	static short poc=0;
	
	for (short i=0;i<(FIR_HP-1);i++)
	{
		Temp_pole[id][i]=Temp_pole[id][i+1];
	}
	
	Temp_pole[id][FIR_HP-1]=*data;
	poc++;
	if (poc==10) poc=0;
	
	
	for (short j=0;j<FIR_HP;j++)
	{
		Temp+=(float)(FIR_HPass[j]*(Temp_pole[id][j]));
	}
	
	*data=(float)Temp;
}


// void HP_IIR(float *x_in,float y_out)
// {
// 	for (short n=0;n<;n++)
// 	{
// 		y_out[n]=
// 	}
// }