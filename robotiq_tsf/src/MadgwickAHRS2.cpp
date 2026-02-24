//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MadgwickAHRS2.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	1000.0f		// sample frequency in Hz //Modified By JP on December 16th 2015
#define betaDef		0.1f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float betanew = betaDef;								// 2 * proportional gain (Kp)
volatile float q0new = 1.0f, q1new = 0.0f, q2new = 0.0f, q3new = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt2(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MadgwickAHRSupdate2(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MadgwickAHRSupdateIMU2(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1new * gx - q2new * gy - q3new * gz);
    qDot2 = 0.5f * (q0new * gx + q2new * gz - q3new * gy);
    qDot3 = 0.5f * (q0new * gy - q1new * gz + q3new * gx);
    qDot4 = 0.5f * (q0new * gz + q1new * gy - q2new * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
        recipNorm = invSqrt2(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
        recipNorm = invSqrt2(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0new * mx;
        _2q0my = 2.0f * q0new * my;
        _2q0mz = 2.0f * q0new * mz;
        _2q1mx = 2.0f * q1new * mx;
        _2q0 = 2.0f * q0new;
        _2q1 = 2.0f * q1new;
        _2q2 = 2.0f * q2new;
        _2q3 = 2.0f * q3new;
        _2q0q2 = 2.0f * q0new * q2new;
        _2q2q3 = 2.0f * q2new * q3new;
        q0q0 = q0new * q0new;
        q0q1 = q0new * q1new;
        q0q2 = q0new * q2new;
        q0q3 = q0new * q3new;
        q1q1 = q1new * q1new;
        q1q2 = q1new * q2new;
        q1q3 = q1new * q3new;
        q2q2 = q2new * q2new;
        q2q3 = q2new * q3new;
        q3q3 = q3new * q3new;

		// Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3new + _2q0mz * q2new + mx * q1q1 + _2q1 * my * q2new + _2q1 * mz * q3new - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3new + my * q0q0 - _2q0mz * q1new + _2q1mx * q2new - my * q1q1 + my * q2q2 + _2q2 * mz * q3new - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2new + _2q0my * q1new + mz * q0q0 + _2q1mx * q3new - mz * q1q1 + _2q2 * my * q3new - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2new * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3new + _2bz * q1new) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2new * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1new * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3new * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2new + _2bz * q0new) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3new - _4bz * q1new) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2new * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2new - _2bz * q0new) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1new + _2bz * q3new) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0new - _4bz * q2new) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3new + _2bz * q1new) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0new + _2bz * q2new) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1new * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = invSqrt2(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
        qDot1 -= betanew * s0;
        qDot2 -= betanew * s1;
        qDot3 -= betanew * s2;
        qDot4 -= betanew * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
    q0new += qDot1 * (1.0f / sampleFreq);
    q1new += qDot2 * (1.0f / sampleFreq);
    q2new += qDot3 * (1.0f / sampleFreq);
    q3new += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
    recipNorm = invSqrt2(q0new * q0new + q1new * q1new + q2new * q2new + q3new * q3new);
    q0new *= recipNorm;
    q1new *= recipNorm;
    q2new *= recipNorm;
    q3new *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU2(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1new * gx - q2new * gy - q3new * gz);
    qDot2 = 0.5f * (q0new * gx + q2new * gz - q3new * gy);
    qDot3 = 0.5f * (q0new * gy - q1new * gz + q3new * gx);
    qDot4 = 0.5f * (q0new * gz + q1new * gy - q2new * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
        recipNorm = invSqrt2(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0new;
        _2q1 = 2.0f * q1new;
        _2q2 = 2.0f * q2new;
        _2q3 = 2.0f * q3new;
        _4q0 = 4.0f * q0new;
        _4q1 = 4.0f * q1new;
        _4q2 = 4.0f * q2new;
        _8q1 = 8.0f * q1new;
        _8q2 = 8.0f * q2new;
        q0q0 = q0new * q0new;
        q1q1 = q1new * q1new;
        q2q2 = q2new * q2new;
        q3q3 = q3new * q3new;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1new - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2new + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3new - _2q1 * ax + 4.0f * q2q2 * q3new - _2q2 * ay;
        recipNorm = invSqrt2(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
        qDot1 -= betanew * s0;
        qDot2 -= betanew * s1;
        qDot3 -= betanew * s2;
        qDot4 -= betanew * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
    q0new += qDot1 * (1.0f / sampleFreq);
    q1new += qDot2 * (1.0f / sampleFreq);
    q2new += qDot3 * (1.0f / sampleFreq);
    q3new += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
    recipNorm = invSqrt2(q0new * q0new + q1new * q1new + q2new * q2new + q3new * q3new);
    q0new *= recipNorm;
    q1new *= recipNorm;
    q2new *= recipNorm;
    q3new *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt2(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
