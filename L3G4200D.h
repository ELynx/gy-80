#ifndef L3G4200D_h_
#define L3G4200D_h_

#include "imusensor.h"

class L3G4200D : public ImuSensor
{
public:
	L3G4200D() = default;
	virtual ~ L3G4200D() = default;
	
	virtual int init();
	virtual int measure(float &, float &, float &);
};

#endif
