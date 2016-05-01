#ifndef imusensor_h_
#define imusensor_h_

#include <Arduino.h>

class ImuSensor
{
public:
	ImuSensor() = default;
	virtual ~ImuSensor() = default;

	virtual int init() = 0;
	virtual int measure(float &, float &, float &) = 0;
};

#endif
