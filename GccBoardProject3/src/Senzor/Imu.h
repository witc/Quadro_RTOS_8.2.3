/*
 * Imu.h
 *
 * Created: 06.05.2016 10:58:01
 *  Author: J
 */ 


#ifndef IMU_H_
#define IMU_H_

// Floating point 3 vector.
typedef struct fp_vector {
	float XYZ[3];
} fp_vect;

// typedef union {
// 	float A[3];
// 	t_fp_vector_xyz Vect;
// } fp_est_vector;
void IMU_Compute(short *GyroAngle, short *Accel,float *uhel, float dt);
void rotate_DCM( fp_vect *v, float *delta);
void acc_calc(short *AccXYZ,float *angle_radian, float dt);
float applyDeadband(float value, float deadband);
void normalizeV(struct fp_vector *src, struct fp_vector *dest);
void HP_Fir(float *data,short id);




#endif /* IMU_H_ */