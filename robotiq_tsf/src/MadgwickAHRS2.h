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
#ifndef MadgwickAHRS2_h
#define MadgwickAHRS2_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float betanew;				// algorithm gain
extern volatile float q0new, q1new, q2new, q3new;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate2(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU2(float gx, float gy, float gz, float ax, float ay, float az);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
