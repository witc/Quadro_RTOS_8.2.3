////=====================================================================================================
//// MadgwickAHRS.c
////=====================================================================================================
////
//// Implementation of Madgwick's IMU and AHRS algorithms.
//// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
////
//// Date			Author          Notes
//// 29/09/2011	SOH Madgwick    Initial release
//// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
////
////=====================================================================================================
//
////---------------------------------------------------------------------------------------------------
//// Header files
//#include <asf.h>
//#include "FreeRTOS_V_8_Header.h"
//#include "MadgwickAHRS.h"
//#include <math.h>
//#include "Imu.h"
//#include "Motor.h"
//#include "stdlib.h"
////---------------------------------------------------------------------------------------------------
//// Definitions
//
//#define sampleFreq	512.0f		// sample frequency in Hz
//#define betaDef		0.1f		// 2 * proportional gain
//
////---------------------------------------------------------------------------------------------------
//// Variable definitions
//
//volatile float beta = betaDef;								// 2 * proportional gain (Kp)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
//volatile float rMat[3][3];
//int16_t accSmooth[3]={0,0,0};
//int16_t AccXYZ[3]={0,0,0};
//int32_t accSum[3]={0,0,0};
//
//uint32_t accTimeSum = 0;        // keep track for integration of acc
//int accSumCount = 0;
//static float accVelScale =0.000000121f;// 9.80665f / acc_1G / 10000.0f;
//volatile float altitude_akce=0;
////---------------------------------------------------------------------------------------------------
//// Function declarations
//
//float invSqrt(float x);
//
////====================================================================================================
//// Functions
//
////---------------------------------------------------------------------------------------------------
//// AHRS algorithm update
//
//void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float delta_t) {
	//float recipNorm;
	//float s0, s1, s2, s3;
	//float qDot1, qDot2, qDot3, qDot4;
	//float hx, hy;
	//float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
//
	//// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	//if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		//MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az,delta_t);
		//return;
	//}
//
	//// Rate of change of quaternion from gyroscope
	//qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	//qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	//qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	//qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
//
	//// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	//if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
//
		//// Normalise accelerometer measurement
		//recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		//ax *= recipNorm;
		//ay *= recipNorm;
		//az *= recipNorm;   
//
		//// Normalise magnetometer measurement
		//recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		//mx *= recipNorm;
		//my *= recipNorm;
		//mz *= recipNorm;
//
		//// Auxiliary variables to avoid repeated arithmetic
		//_2q0mx = 2.0f * q0 * mx;
		//_2q0my = 2.0f * q0 * my;
		//_2q0mz = 2.0f * q0 * mz;
		//_2q1mx = 2.0f * q1 * mx;
		//_2q0 = 2.0f * q0;
		//_2q1 = 2.0f * q1;
		//_2q2 = 2.0f * q2;
		//_2q3 = 2.0f * q3;
		//_2q0q2 = 2.0f * q0 * q2;
		//_2q2q3 = 2.0f * q2 * q3;
		//q0q0 = q0 * q0;
		//q0q1 = q0 * q1;
		//q0q2 = q0 * q2;
		//q0q3 = q0 * q3;
		//q1q1 = q1 * q1;
		//q1q2 = q1 * q2;
		//q1q3 = q1 * q3;
		//q2q2 = q2 * q2;
		//q2q3 = q2 * q3;
		//q3q3 = q3 * q3;
//
		//// Reference direction of Earth's magnetic field
		//hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		//hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		//_2bx = sqrt(hx * hx + hy * hy);
		//_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		//_4bx = 2.0f * _2bx;
		//_4bz = 2.0f * _2bz;
//
		//// Gradient decent algorithm corrective step
		//s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		//s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		//s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		//s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		//recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		//s0 *= recipNorm;
		//s1 *= recipNorm;
		//s2 *= recipNorm;
		//s3 *= recipNorm;
//
		//// Apply feedback step
		//qDot1 -= beta * s0;
		//qDot2 -= beta * s1;
		//qDot3 -= beta * s2;
		//qDot4 -= beta * s3;
	//}
//
	//// Integrate rate of change of quaternion to yield quaternion
	//q0 += qDot1 * (1.0f * delta_t);
	//q1 += qDot2 * (1.0f * delta_t);
	//q2 += qDot3 * (1.0f * delta_t);
	//q3 += qDot4 * (1.0f * delta_t);
//
	//// Normalise quaternion
	//recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	//q0 *= recipNorm;
	//q1 *= recipNorm;
	//q2 *= recipNorm;
	//q3 *= recipNorm;
//}
//
////---------------------------------------------------------------------------------------------------
//// IMU algorithm update
//
//void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float  delta_t) {
	//float recipNorm;
	//float s0, s1, s2, s3;
	//float qDot1, qDot2, qDot3, qDot4;
	//float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
//
	//// Rate of change of quaternion from gyroscope
	//qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	//qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	//qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	//qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
//
	//// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	//if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
//
		//// Normalise accelerometer measurement
		//recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		//ax *= recipNorm;
		//ay *= recipNorm;
		//az *= recipNorm;   
//
		//// Auxiliary variables to avoid repeated arithmetic
		//_2q0 = 2.0f * q0;
		//_2q1 = 2.0f * q1;
		//_2q2 = 2.0f * q2;
		//_2q3 = 2.0f * q3;
		//_4q0 = 4.0f * q0;
		//_4q1 = 4.0f * q1;
		//_4q2 = 4.0f * q2;
		//_8q1 = 8.0f * q1;
		//_8q2 = 8.0f * q2;
		//q0q0 = q0 * q0;
		//q1q1 = q1 * q1;
		//q2q2 = q2 * q2;
		//q3q3 = q3 * q3;
//
		//// Gradient decent algorithm corrective step
		//s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		//s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		//s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		//s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		//recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		//s0 *= recipNorm;
		//s1 *= recipNorm;
		//s2 *= recipNorm;
		//s3 *= recipNorm;
//
		//// Apply feedback step
		//qDot1 -= beta * s0;
		//qDot2 -= beta * s1;
		//qDot3 -= beta * s2;
		//qDot4 -= beta * s3;
	//}
//
	//// Integrate rate of change of quaternion to yield quaternion
	//q0 += qDot1 * (1.0f * delta_t);
	//q1 += qDot2 * (1.0f * delta_t);
	//q2 += qDot3 * (1.0f * delta_t);
	//q3 += qDot4 * (1.0f * delta_t);
//
	//// Normalise quaternion
	//recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	//q0 *= recipNorm;
	//q1 *= recipNorm;
	//q2 *= recipNorm;
	//q3 *= recipNorm;
//}
//
////---------------------------------------------------------------------------------------------------
//// Fast inverse square-root
//// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
//
//float invSqrt(float x) {
	//float halfx = 0.5f * x;
	//float y = x;
	//long i = *(long*)&y;
	//i = 0x5f3759df - (i>>1);
	//y = *(float*)&i;
	//y = y * (1.5f - (halfx * y * y));
	//return y;
//}
//
//void imuComputeRotationMatrix(void)
//{
	//float q1q1 = sq(q1);
	//float q2q2 = sq(q2);
	//float q3q3 = sq(q3);
	//
	//float q0q1 = q0 * q1;
	//float q0q2 = q0 * q2;
	//float q0q3 = q0 * q3;
	//float q1q2 = q1 * q2;
	//float q1q3 = q1 * q3;
	//float q2q3 = q2 * q3;
//
	//rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
	//rMat[0][1] = 2.0f * (q1q2 + -q0q3);
	//rMat[0][2] = 2.0f * (q1q3 - -q0q2);
//
	//rMat[1][0] = 2.0f * (q1q2 - -q0q3);
	//rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
	//rMat[1][2] = 2.0f * (q2q3 + -q0q1);
//
	//rMat[2][0] = 2.0f * (q1q3 + -q0q2);
	//rMat[2][1] = 2.0f * (q2q3 - -q0q1);
	//rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
//}
//
//
//void imuUpdateEulerAngles(void)
//{
//// 	/* Compute pitch/roll angles */
//// 	attitude.values.roll = lrintf(atan2_approx(rMat[2][1], rMat[2][2]) * (1800.0f / M_PIf));
//// 	attitude.values.pitch = lrintf(((0.5f * M_PIf) - acos_approx(-rMat[2][0])) * (1800.0f / M_PIf));
//// 	attitude.values.yaw = lrintf((-atan2_approx(rMat[1][0], rMat[0][0]) * (1800.0f / M_PIf) + magneticDeclination));
//// 
//// 	if (attitude.values.yaw < 0)
//// 	attitude.values.yaw += 3600;
//// 
//// 	/* Update small angle state */
//// 	if (rMat[2][2] > smallAngleCosZ) {
//// 		ENABLE_STATE(SMALL_ANGLE);
//// 		} else {
//// 		DISABLE_STATE(SMALL_ANGLE);
//// 	}
//}
//
//
//// rotate acc into Earth frame and calculate acceleration in it
//void imuCalculateAcceleration(float deltaT)
//{
	//static int32_t accZoffset = 0;
	//static float accz_smooth = 0;
	//t_fp_est_vector accel_ned;
//
	//accel_ned.Vect.X = accSmooth[0];
	//accel_ned.Vect.Y = accSmooth[1];
	//accel_ned.Vect.Z = accSmooth[2];
//
	//imuTransformVectorBodyToEarth(&accel_ned);
	//
	///* pro pøípad že quadro vzlítá z ruky*/
//// 
//// 	if (imuRuntimeConfig->acc_unarmedcal == 1) {
//// 		if (!ARMING_FLAG(ARMED)) {
//// 			accZoffset -= accZoffset / 64;
//// 			accZoffset += accel_ned.V.Z;
//// 		}
//// 		accel_ned.V.Z -= accZoffset / 64;  // compensate for gravitation on z-axis
//// 	} else
	//accel_ned.Vect.Z -= acc_1G;
	//
	//accz_smooth=filterApplyPt1(accz_smooth,&Filters[ACC_SMOTH_Z],20,deltaT);
	////accz_smooth = accz_smooth + (dT / (fc_acc + dT)) * (accel_ned.V.Z - accz_smooth); // low pass filter
//
	//// apply Deadband to reduce integration drift and vibration influence
	//accSum[0] += applyDeadband(lrintf(accel_ned.Vect.X), 5);
	//accSum[1] += applyDeadband(lrintf(accel_ned.Vect.Y), 5);
	//accSum[2] += applyDeadband(lrintf(accz_smooth), 5);
//
	//// sum up Values for later integration to get velocity and distance
	//accSumCount++;
//}
//
//void imuTransformVectorBodyToEarth(t_fp_est_vector * v)
//{
	//float x,y,z;
//
	///* From body frame to earth frame */
	//x = rMat[0][0] * v->Vect.X + rMat[0][1] * v->Vect.Y + rMat[0][2] * v->Vect.Z;
	//y = rMat[1][0] * v->Vect.X + rMat[1][1] * v->Vect.Y + rMat[1][2] * v->Vect.Z;
	//z = rMat[2][0] * v->Vect.X + rMat[2][1] * v->Vect.Y + rMat[2][2] * v->Vect.Z;
//
	//v->Vect.X = x;
	//v->Vect.Y = -y;
	//v->Vect.Z = z;
//}
//
//int32_t applyDeadband(int32_t value, int32_t deadband)
//{
	//if (abs(value) < deadband) {
		//value = 0;
		//} else if (value > 0) {
		//value -= deadband;
		//} else if (value < 0) {
		//value += deadband;
	//}
	//return value;
//}
//
//
//void calculateEstimatedAltitude(float currentTime)
//{
	//static uint32_t previousTime;
	//uint32_t dTime;
	//float dt;
	//float vel_acc;
	//int32_t vel_tmp;
	//float accZ_tmp;
	//static float accZ_old = 0.0f;
	//static float vel = 0.0f;
	//static float accAlt = 0.0f;
	//static int32_t lastBaroAlt;
	////static int32_t lastBaroAlt;
	//
	////BaroAlt = baroCalculateAltitude();
	//
	//dt = currentTime ;
//
	//// Integrator - velocity, cm/sec
	//if (accSumCount) {
		//accZ_tmp = (float)accSum[2] / (float)accSumCount;
		//} else {
		//accZ_tmp = 0;
	//}
	//vel_acc = accZ_tmp * accVelScale * (float)currentTime;
//
	//// Integrator - Altitude in cm
	//accAlt += (vel_acc * 0.5f) * dt + vel * dt;                                                                 // integrate velocity to get distance (x= a/2 * t^2)
	////accAlt = accAlt * barometerConfig->baro_cf_alt + (float)BaroAlt * (1.0f - barometerConfig->baro_cf_alt);    // complementary filter for altitude estimation (baro & acc)
	//vel += vel_acc;
	//altitude_akce=accAlt;
	//
//}
//
//// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
//void rotate_DCM(struct fp_vector *vect, float *delta)
//{
	//struct fp_vector v_tmp = *vect;
//
	//// This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
	//float mat[3][3];
	//float cosx, sinx, cosy, siny, cosz, sinz;
	//float coszcosx, sinzcosx, coszsinx, sinzsinx;
//
	//cosx = cosf(delta[0]);
	//sinx = sinf(delta[0]);
	//cosy = cosf(delta[1]);
	//siny = sinf(delta[1]);
	//cosz = cosf(delta[2]);
	//sinz = sinf(delta[2]);
	//
//// 	cosx = 1;
//// 	sinx = (delta[0]);
//// 	cosy = 1;
//// 	siny = (delta[1]);
//// 	cosz = 1;
//// 	sinz = (delta[2]);
//
//
	//coszcosx = cosz * cosx;
	//sinzcosx = sinz * cosx;
	//coszsinx = sinx * cosz;
	//sinzsinx = sinx * sinz;
//
	//mat[0][0] = cosz * cosy;
	//mat[0][1] = -cosy * sinz;
	//mat[0][2] = siny;
	//mat[1][0] = sinzcosx + (coszsinx * siny);
	//mat[1][1] = coszcosx - (sinzsinx * siny);
	//mat[1][2] = -sinx * cosy;
	//mat[2][0] = (sinzsinx) - (coszcosx * siny);
	//mat[2][1] = (coszsinx) + (sinzcosx * siny);
	//mat[2][2] = cosy * cosx;
//
	//vect->X = v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];
	//vect->Y = v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
	//vect->Z = v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
//}
//
//// rotate acc into Earth frame and calculate acceleration in it
//void acc_calc(float *angle_radian, float dt)
//{
	//static int32_t accZoffset = 0;
	//static float accz_smooth = 0;
	//float rpy[3];
	//t_fp_est_vector accel_ned;
	//// the accel values have to be rotated into the earth frame
	//rpy[0] = -(float)angle_radian[0];
	//rpy[1] = -(float)angle_radian[1];
	//rpy[2] = -(float)angle_radian[2];
//
	//accel_ned.Vect.X = AccXYZ[0];
	//accel_ned.Vect.Y = AccXYZ[1];
	//accel_ned.Vect.Z = AccXYZ[2];
//
	//rotate_DCM(&accel_ned.Vect, rpy);
//// 
//// 	if (cfg.acc_unarmedcal == 1)
//// 	{
//// 		if (!f.ARMED) {
//// 			accZoffset -= accZoffset / 64;
//// 			accZoffset += accel_ned.V.Z;
//// 		}
//// 		accel_ned.V.Z -= accZoffset / 64;  // compensate for gravitation on z-axis
//// 	} else
	//accel_ned.Vect.Z -= acc_1G;
//
	////accz_smooth = accz_smooth + (dT / (fc_acc + dT)) * (accel_ned.V.Z - accz_smooth); // low pass filter- accz_smooth
	//accz_smooth=filterApplyPt1((accel_ned.Vect.Z ),&Filters[ACC_SMOTH_Z],10,dt);
	//
	//// apply Deadband to reduce integration drift and vibration influence and
	//// sum up Values for later integration to get velocity and distance
	//accSum[0] = applyDeadband(lrintf(accel_ned.Vect.X), 50);
	//accSum[1] = applyDeadband(lrintf(accel_ned.Vect.Y), 50);
	//accSum[2] = applyDeadband(lrintf(accz_smooth), 50);
//
//// 	accTimeSum += deltaT;
//// 	accSumCount++;
//}
//
//int getEstimatedAltitude(float dt)
//{
	//static uint32_t previousT;
	//uint32_t dTime;
	//int32_t error;
	//int32_t baroVel;
	//int32_t vel_tmp;
	//int32_t BaroAlt_tmp;
	//int32_t setVel;
	//float vel_acc;
	//float accZ_tmp;
	//static float accZ_old = 0.0f;
	//static float vel = 0.0f;
	//static float accAlt = 0.0f;
	//static int32_t lastBaroAlt;
	//static int32_t baroGroundAltitude = 0;
	//static int32_t baroGroundPressure = 0;
//
	//
//
//// 	if (calibratingB > 0) {
//// 		baroGroundPressure -= baroGroundPressure / 8;
//// 		baroGroundPressure += baroPressureSum / (cfg.baro_tab_size - 1);
//// 		baroGroundAltitude = (1.0f - powf((baroGroundPressure / 8) / 101325.0f, 0.190295f)) * 4433000.0f;
//// 
//// 		vel = 0;
//// 		accAlt = 0;
//// 		calibratingB--;
//// 	}
//
	//// calculates height from ground via baro readings
	//// see: https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Baro/AP_Baro.cpp#L140
//// 	BaroAlt_tmp = lrintf((1.0f - powf((float)(baroPressureSum / (cfg.baro_tab_size - 1)) / 101325.0f, 0.190295f)) * 4433000.0f); // in cm
//// 	BaroAlt_tmp -= baroGroundAltitude;
//// 	BaroAlt = lrintf((float)BaroAlt * cfg.baro_noise_lpf + (float)BaroAlt_tmp * (1.0f - cfg.baro_noise_lpf)); // additional LPF to reduce baro noise
//
	//// calculate sonar altitude only if the sonar is facing downwards(<25deg)
//// 	if (tiltAngle > 250)
//// 	sonarAlt = -1;
//// 	else
//// 	sonarAlt = sonarAlt * (900.0f - tiltAngle) / 900.0f;
//// 
//// 	// do sonarAlt and baroAlt fusion
//// 	if (sonarAlt > 0 && sonarAlt < 200) {
//// 		baroAlt_offset = BaroAlt - sonarAlt;
//// 		BaroAlt = sonarAlt;
//// 		} else {
//// 		BaroAlt -= baroAlt_offset;
//// 		if (sonarAlt > 0) {
//// 			sonarTransition = (300 - sonarAlt) / 100.0f;
//// 			BaroAlt = sonarAlt * sonarTransition + BaroAlt * (1.0f - sonarTransition);
//// 		}
//// 	}
//// 
//// 	dt = accTimeSum * 1e-6f; // delta acc reading time in seconds
//
	//// Integrator - velocity, cm/sec
	//accZ_tmp = (float)accSum[2] / (float)accSumCount;
	//vel_acc = accZ_tmp * accVelScale * (float)accTimeSum;
//
	//// Integrator - Altitude in cm
	//accAlt += (vel_acc * 0.5f) * dt + vel * dt;                                         // integrate velocity to get distance (x= a/2 * t^2)
	////accAlt = accAlt * 0.95f + (float)BaroAlt * (1.0f - 0.95f);      // complementary filter for altitude estimation (baro & acc)
//
	//// when the sonar is in his best range
//// 	if (sonarAlt > 0 && sonarAlt < 200)
//// 	EstAlt = BaroAlt;
//// 	else
//// 	EstAlt = accAlt;
//
	//vel += vel_acc;
//
//// 	#if 0
//// 	debug[0] = accSum[2] / accSumCount; // acceleration
//// 	debug[1] = vel;                     // velocity
//// 	debug[2] = accAlt;                  // height
//// 	#endif
//
	//accSum_reset();
//
//// 	baroVel = (BaroAlt - lastBaroAlt) * 1000000.0f / dTime;
//// 	lastBaroAlt = BaroAlt;
//// 
//// 	baroVel = constrain(baroVel, -1500, 1500);    // constrain baro velocity +/- 1500cm/s
//// 	baroVel = applyDeadband(baroVel, 10);         // to reduce noise near zero
//
	//// apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
	//// By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
//// 	vel = vel * cfg.baro_cf_vel + baroVel * (1 - cfg.baro_cf_vel);
//// 	vel_tmp = lrintf(vel);
//// 
//// 	// set vario
//// 	vario = applyDeadband(vel_tmp, 5);
//// 
//// 	if (tiltAngle < 800) { // only calculate pid if the copters thrust is facing downwards(<80deg)
//// 		// Altitude P-Controller
//// 		if (!velocityControl) {
//// 			error = constrain(AltHold - EstAlt, -500, 500);
//// 			error = applyDeadband(error, 10);       // remove small P parametr to reduce noise near zero position
//// 			setVel = constrain((cfg.P8[PIDALT] * error / 128), -300, +300); // limit velocity to +/- 3 m/s
//// 			} else {
//// 			setVel = setVelocity;
//// 		}
//// 
//// 		// Velocity PID-Controller
//// 		// P
//// 		error = setVel - vel_tmp;
//// 		BaroPID = constrain((cfg.P8[PIDVEL] * error / 32), -300, +300);
//// 
//// 		// I
//// 		errorVelocityI += (cfg.I8[PIDVEL] * error);
//// 		errorVelocityI = constrain(errorVelocityI, -(8196 * 200), (8196 * 200));
//// 		BaroPID += errorVelocityI / 8196;     // I in the range of +/-200
//// 
//// 		// D
//// 		BaroPID -= constrain(cfg.D8[PIDVEL] * (accZ_tmp + accZ_old) / 512, -150, 150);
//// 
//// 		} else {
//// 		BaroPID = 0;
//// 	}
//
	//accZ_old = accZ_tmp;
//
	//return 1;
//}
//
//void accSum_reset(void)
//{
	//accSum[0] = 0;
	//accSum[1] = 0;
	//accSum[2] = 0;
	//accSumCount = 0;
	//accTimeSum = 0;
//}
//
//// baseflight calculation by Luggi09 originates from arducopter
 //int16_t calculateHeading(t_fp_est_vector *vec,float pitch, float roll)
//{
	//int16_t head;
//
	//float cosineRoll = cosf(pitch);
	//float sineRoll = sinf(pitch);
	//float cosinePitch = cosf(roll);
	//float sinePitch = sinf(roll);
	//float Xh = vec->A[0] * cosinePitch + vec->A[1] * sineRoll * sinePitch + vec->A[2] * sinePitch * cosineRoll;
	//float Yh = vec->A[1] * cosineRoll - vec->A[2] * sineRoll;
	//float hd = ((atan2f(Yh, Xh) * 180.0f / M_PI) + 3);//+3 = mag incilination
	//head = lrintf(hd);
	//if (head < 0)
	//head += 360;
//
	//return head;
//}
//
//// Normalize a vector
//void normalizeV(struct fp_vector *src, struct fp_vector *dest)
//{
	//float length;
//
	//length = sqrtf(src->X * src->X + src->Y * src->Y + src->Z * src->Z);
	//if (length != 0) {
		//dest->X = src->X / length;
		//dest->Y = src->Y / length;
		//dest->Z = src->Z / length;
	//}
//}
//
//
//// Definitions
//
//#define Kp 2.0f			// proportional gain governs rate of convergence to accelerometer/magnetometer
//#define Ki 0.005f		// integral gain governs rate of convergence of gyroscope biases
//#define halfT 0.5f		// half the sample period
//
////---------------------------------------------------------------------------------------------------
//// Variable definitions
//
////float q0 = 1, q1 = 0, q2 = 0, q3 = 0;	// quaternion elements representing the estimated orientation
//float exInt = 0, eyInt = 0, ezInt = 0;	// scaled integral error
//
////====================================================================================================
//// Function
////====================================================================================================
//
//void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az, float halfdt) {
	//float norm;
	//float vx, vy, vz;
	//float ex, ey, ez;
	//
	//// normalise the measurements
	//norm = sqrt(ax*ax + ay*ay + az*az);
	//ax = ax / norm;
	//ay = ay / norm;
	//az = az / norm;
	//
	//// estimated direction of gravity
	//vx = 2*(q1*q3 - q0*q2);
	//vy = 2*(q0*q1 + q2*q3);
	//vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	//
	//// error is sum of cross product between reference direction of field and direction measured by sensor
	//ex = (ay*vz - az*vy);
	//ey = (az*vx - ax*vz);
	//ez = (ax*vy - ay*vx);
	//
	//// integral error scaled integral gain
	//exInt = exInt + ex*Ki;
	//eyInt = eyInt + ey*Ki;
	//ezInt = ezInt + ez*Ki;
	//
	//// adjusted gyroscope measurements
	//gx = gx + Kp*ex + exInt;
	//gy = gy + Kp*ey + eyInt;
	//gz = gz + Kp*ez + ezInt;
	//
	//// integrate quaternion rate and normalise
	//q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfdt;
	//q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfdt;
	//q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfdt;
	//q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfdt;
	//
	//// normalise quaternion
	//norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	//q0 = q0 / norm;
	//q1 = q1 / norm;
	//q2 = q2 / norm;
	//q3 = q3 / norm;
//}
////====================================================================================================
//// END OF CODE
////====================================================================================================
