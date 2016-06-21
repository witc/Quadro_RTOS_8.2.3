//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#include <asf.h>
#include <stdlib.h>
#include <arm_math.h>

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations
typedef struct  
{
	double pitch;
	double roll;
	double yaw;
}Heading;



/**
 * @brief Quaternion.  This library uses the conversion of placing the 'w'
 * element as the first element.  Other implementations may place the 'w'
 * element as the last element.
 */
typedef struct{
        float w;
        float x;
        float y;
        float z;
   
} Quaternion;

/**
 * @brief Definition of M_PI.  Some compilers may not define this in math.h.
 */
#ifndef M_PI
#define    M_PI 3.14159265358979323846
#endif

/**
 * @brief Macro for converting radians to degrees.
 */
#define RADIANS_TO_DEGREES(radians) ((float)(radians) * (180.0f / M_PI))

/**
 * @brief Euler angles union.  The Euler angles are in the Aerospace sequence
 * also known as the ZYX sequence.
 */
typedef  struct {
        float roll;
        float pitch;
        float yaw;
 } EulerAngles;





void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,EulerAngles *Angle);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,EulerAngles *Angle);
void  QuaternionToEulerAngles(Quaternion quaternion,EulerAngles *eulerAngles);
float invSqrt(float x);



#endif
//=====================================================================================================
// End of file
//=====================================================================================================