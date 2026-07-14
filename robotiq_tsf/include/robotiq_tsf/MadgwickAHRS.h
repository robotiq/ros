// MadgwickAHRS.h
//
// Class wrapper around Madgwick's IMU AHRS algorithm, plus quaternion-math
// helpers used by the ROS node to compute relative ("zeroed") orientation.
//
// Original algorithm: Sebastian O.H. Madgwick, 2011.
// Refactored 2026 to: instance state (no globals), measured dt, accelerometer
// magnitude gating, and accel-seeded initialization.

#ifndef ROBOTIQ_TSF_MADGWICK_AHRS_H
#define ROBOTIQ_TSF_MADGWICK_AHRS_H

class MadgwickFilter
{
public:
  explicit MadgwickFilter(float beta = 0.041f);

  void reset();
  void setBeta(float beta);
  void setAccelGate(float lo, float hi);

  // Seed the quaternion so the gravity vector in the body frame matches the
  // supplied accelerometer reading (any units; only the direction is used).
  // Yaw is set to zero. Use the calibration-time accel mean.
  void initFromAccel(float ax, float ay, float az);

  // gyro in rad/s, accel in any consistent unit (normalised internally),
  // dt in seconds (use the measured interval between samples).
  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);

  void getQuaternion(float& q0, float& q1, float& q2, float& q3) const;
  void getEulerDeg(float& roll, float& pitch, float& yaw) const;

private:
  float q0_, q1_, q2_, q3_;
  float beta_;
  float accel_gate_lo_;  // expected |a| ≈ 1.0 g when stationary
  float accel_gate_hi_;
};

// --- quaternion-math helpers (free functions) ---------------------------------
// Convention: q = [w, x, y, z] = [q0, q1, q2, q3].
// Euler order matches the Madgwick implementation's ZYX Tait-Bryan extraction
// (yaw around Z, pitch around Y, roll around X), angles in radians for the
// builder helpers, degrees for quatToEulerDeg.

void quatMul(const float a[4], const float b[4], float out[4]);
void quatConj(const float q[4], float out[4]);
void quatNormalize(float q[4]);
void quatFromAxisX(float angle_rad, float out[4]);
void quatFromAxisY(float angle_rad, float out[4]);
void quatFromAxisZ(float angle_rad, float out[4]);
void quatToEulerDeg(const float q[4], float& roll, float& pitch, float& yaw);
void quatToEulerRad(const float q[4], float& roll, float& pitch, float& yaw);

#endif  // ROBOTIQ_TSF_MADGWICK_AHRS_H
