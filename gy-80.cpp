#include "gy-80.h"

#include <i2c_t3.h>

#include "madgwick.h"

int Gy80::init()
{
	/*
	We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
	We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ to 400000L /twi.h utility file.
	The Teensy has no internal pullups and we are using the Wire.begin function of the i2c_t3.h library
	to select 400 Hz i2c speed.
	*/

	Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
	//delay(4000);

	lastUpdate = micros();

	quart.q1() = 1.0f;
	quart.q2() = 0.0f;
	quart.q3() = 0.0f;
	quart.q4() = 0.0f;

	if (acel.init() != 0)
		return -1;

	if (gyro.init() != 0)
		return -2;
	
	if (magn.init() != 0)
		return -3;

	if (pres.init() != 0)
		return -4;

	return 0;
}

ImuData Gy80::sense()
{
	FilterInput filterInput;

	const uint32_t now = micros();
	filterInput.deltaT = (float(now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
	lastUpdate = now;

	int a = acel.measure(filterInput.ax(), filterInput.ay(), filterInput.az()); //m/s^2
	int g = gyro.measure(filterInput.gx(), filterInput.gy(), filterInput.gz()); //rad/s/s
	int m = magn.measure(filterInput.mx(), filterInput.my(), filterInput.mz()); //mGauss

	ImuData error;

	if (a < 0)
	{
		error.ax() = -1.0f;
	}
	
	if (g < 0)
	{
		error.ay() = -1.0f;
	}
	
	if (m < 0)
	{
		error.az() = -1.0f;
	}
	
	if (a < 0 || g < 0 || m < 0)
	{
		return error;
	}
	
	//if not ready then make NaNs, code in filter will skip correction
	if (m < 0)
	{
		filterInput.mx() = nan("");
		filterInput.my() = nan("");
		filterInput.mz() = nan("");
	}
	
	//TODO rotate values to match directions
	//TODO apply magnetometer bias
	
	MadgwickQuaternionUpdate(quart, filterInput);

	// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
	// In this coordinate system, the positive z-axis is down toward Earth. 
	// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
	// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
	// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
	// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
	// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
	// applied in the correct order which for this configuration is yaw, pitch, and then roll.
	// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.

	float pitch, yaw, roll;
	
	yaw   = atan2(2.0f * (quart.q2() * quart.q3() + quart.q1() * quart.q4()), quart.q1() * quart.q1() + quart.q2() * quart.q2() - quart.q3() * quart.q3() - quart.q4() * quart.q4());   
	pitch = -asin(2.0f * (quart.q2() * quart.q4() - quart.q1() * quart.q3()));
	roll  = atan2(2.0f * (quart.q1() * quart.q2() + quart.q3() * quart.q4()), quart.q1() * quart.q1() - quart.q2() * quart.q2() - quart.q3() * quart.q3() + quart.q4() * quart.q4());
	pitch *= 180.0f / PI;
	yaw   *= 180.0f / PI; 
	yaw   -= 10.0f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
	roll  *= 180.0f / PI;
	
	ImuData result;
	
	result.ax() = yaw;
	result.ay() = pitch;
	result.az() = roll;
	
	return result;
}
