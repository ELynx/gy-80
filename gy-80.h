#ifndef gy80_h_
#define gy80_h_

#include <Arduino.h>

#include "quart.h"
#include "imudata.h"

#include "ADXL345.h"
#include "L3G4200D.h"
#include "HMC5883L.h"
#include "BMP085.h"

class Gy80
{
public:
	Gy80() = default;

	Gy80 (const Gy80 &) = delete;
	Gy80 & operator = (const Gy80 &) = delete;

	int init();
	ImuData sense();

protected:
	uint32_t lastUpdate;
	Quart quart;

	ADXL345  acel;
	L3G4200D gyro;
	HMC5883L magn;
	BMP085   pres;
};

#endif
