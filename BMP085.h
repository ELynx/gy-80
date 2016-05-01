#ifndef BMP085_h_
#define BMP085_h_

#include "imusensor.h"

class BMP085 : public ImuSensor
{
public:
	BMP085() = default;
	virtual ~BMP085() = default;
	
	virtual int init();
	virtual int measure(float &, float &, float &);
};

#endif
