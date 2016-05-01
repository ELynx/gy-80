#include "madgwick.h"

#include "mathhelp.h"

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.

#define AX input.ax()
#define AY input.ay()
#define AZ input.az()
#define GX input.gx()
#define GY input.gy()
#define GZ input.gz()
#define MX input.mx()
#define MY input.my()
#define MZ input.mz()

void MadgwickQuaternionUpdate(Quart& quart, FilterInput input)
{
	// There is a tradeoff in the beta parameter between accuracy and response speed.
	// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
	// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
	// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
	// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
	// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
	// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
	// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.

	// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
	constexpr float gyroMeasError = deg2rad(40.0f); // gyroscope measurement error in rads/s
	constexpr float beta = sqrt(3.0f / 4.0f) * gyroMeasError;

	// local copies of previous values
	float q1 = quart.q1(), q2 = quart.q2(), q3 = quart.q3(), q4 = quart.q4(); 

	// auxiliary variables to avoid repeated arithmetic
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// normalise accelerometer measurement
	if (!normalize(AX, AY, AZ))
	{
		return;
	}

	// normalise magnetometer measurement
	if (!normalize(MX, MY, MZ))
	{
		return;
	}

	// reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * MX;
	_2q1my = 2.0f * q1 * MY;
	_2q1mz = 2.0f * q1 * MZ;
	_2q2mx = 2.0f * q2 * MX;

	hx = MX * q1q1 - _2q1my * q4 + _2q1mz * q3 + MX * q2q2 + _2q2 * MY * q3 + _2q2 * MZ * q4 - MX * q3q3 - MX * q4q4;
	hy = _2q1mx * q4 + MY * q1q1 - _2q1mz * q2 + _2q2mx * q3 - MY * q2q2 + MY * q3q3 + _2q3 * MZ * q4 - MY * q4q4;

	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + MZ * q1q1 + _2q2mx * q4 - MZ * q2q2 + _2q3 * MY * q4 - MZ * q3q3 + MZ * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - AX) + _2q2 * (2.0f * q1q2 + _2q3q4 - AY) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - MX) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - MY) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - MZ);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - AX) + _2q1 * (2.0f * q1q2 + _2q3q4 - AY) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - AZ) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - MX) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - MY) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - MZ);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - AX) + _2q4 * (2.0f * q1q2 + _2q3q4 - AY) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - AZ) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - MX) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - MY) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - MZ);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - AX) + _2q3 * (2.0f * q1q2 + _2q3q4 - AY) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - MX) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - MY) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - MZ);

	// normalise step magnitude
	bool magOk = normalize(s1, s2, s3, s4);
	
	// if magnetometer measurement is missing, then don't correct
	if (!magOk)
	{
		s1 = 0.0f;
		s2 = 0.0f;
		s3 = 0.0f;
		s4 = 0.0f;
	}

	// compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * GX - q3 * GY - q4 * GZ) - beta * s1;
	qDot2 = 0.5f * ( q1 * GX + q3 * GZ - q4 * GY) - beta * s2;
	qDot3 = 0.5f * ( q1 * GY - q2 * GZ + q4 * GX) - beta * s3;
	qDot4 = 0.5f * ( q1 * GZ + q2 * GY - q3 * GX) - beta * s4;

	// integrate to yield quaternion
	q1 += qDot1 * input.deltaT;
	q2 += qDot2 * input.deltaT;
	q3 += qDot3 * input.deltaT;
	q4 += qDot4 * input.deltaT;

	// normalise quaternion
	if (!normalize(q1, q2, q3, q4))
	{
		return;
	}

	quart.q1() = q1;
	quart.q2() = q2;
	quart.q3() = q3;
	quart.q4() = q4;
}

#undef AX
#undef AY
#undef AZ
#undef GX
#undef GY
#undef GZ
#undef MX
#undef MY
#undef MZ
