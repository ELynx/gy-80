#ifndef ADXL345_h_
#define ADXL345_h_

#include "imusensor.h"

class ADXL345 : public ImuSensor
{
public:
	ADXL345() = default;
	virtual ~ADXL345() = default;
	
	virtual int init();
	virtual int measure(float &ax, float &ay, float &az);
};

#endif
