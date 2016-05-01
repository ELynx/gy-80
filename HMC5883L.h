#ifndef HMC5883L_h_
#define HMC5883L_h_

#include "imusensor.h"

class HMC5883L : public ImuSensor
{
public:
	HMC5883L() = default;
	virtual ~HMC5883L() = default;
	
	virtual int init();
	virtual int measure(float &, float &, float &);
};

#endif
