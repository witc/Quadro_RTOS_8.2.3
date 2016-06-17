////=====================================================================================================
//// MadgwickAHRS.h
////=====================================================================================================
////
//// Implementation of Madgwick's IMU and AHRS algorithms.
//// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
////
//// Date			Author          Notes
//// 29/09/2011	SOH Madgwick    Initial release
//// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
////
////=====================================================================================================
//#ifndef MadgwickAHRS_h
//#define MadgwickAHRS_h
//
////----------------------------------------------------------------------------------------------------
//// Variable declaration
//
//extern volatile float beta;				// algorithm gain
//extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
//extern volatile float rMat[3][3];
//extern volatile float altitude_akce;
//
//// Floating point 3 vector.
//typedef struct fp_vector {
	//float X;
	//float Y;
	//float Z;
//} t_fp_vector_xyz;
//
//typedef union {
	//float A[3];
	//t_fp_vector_xyz Vect;
//} t_fp_est_vector;
//
//typedef struct accDeadband_s {
	//uint8_t xy;                 // set the acc deadband for xy-Axis
	//uint8_t z;                  // set the acc deadband for z-Axis, this ignores small accelerations
//} accDeadband_t;
//
//typedef struct imuRuntimeConfig_s {
	//uint8_t acc_cut_hz;
	//uint8_t acc_unarmedcal;
	//float dcm_ki;
	//float dcm_kp;
	//uint8_t small_angle;
//} imuRuntimeConfig_t;
//
////---------------------------------------------------------------------------------------------------
//// Function declarations
//
//void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float delta_t);
//void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float delta_t);
//void imuComputeRotationMatrix(void);
//void imuUpdateEulerAngles(void);
//void imuCalculateAcceleration(float deltaT);
//void imuTransformVectorBodyToEarth(t_fp_est_vector * v);
//int32_t applyDeadband(int32_t value, int32_t deadband);
//void calculateEstimatedAltitude(float currentTime);
//void acc_calc(float *angle_radian, float dt);
//void rotate_DCM(struct fp_vector *v, float *delta);
//int getEstimatedAltitude(float dt);
//void accSum_reset(void);
//void normalizeV(struct fp_vector *src, struct fp_vector *dest);
 //int16_t calculateHeading(t_fp_est_vector *vec,float pitch, float roll);
//
//
//
//
//
//#endif
////=====================================================================================================
//// End of file
////=====================================================================================================
