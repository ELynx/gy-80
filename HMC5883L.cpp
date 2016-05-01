#include "HMC5883L.h"

#include "i2chelp.h"

#define HMC5883L_ADDRESS	0x1E
#define HMC5883L_CONFIG_A	0x00
#define HMC5883L_CONFIG_B	0x01
#define HMC5883L_MODE		0x02
#define HMC5883L_OUT_X_H	0x03
#define HMC5883L_OUT_X_L	0x04
#define HMC5883L_OUT_Z_H	0x05
#define HMC5883L_OUT_Z_L	0x06
#define HMC5883L_OUT_Y_H	0x07
#define HMC5883L_OUT_Y_L	0x08
#define HMC5883L_STATUS		0x09
#define HMC5883L_IDA		0x0A
#define HMC5883L_IDB		0x0B
#define HMC5883L_IDC		0x0C
#define HMC5883L_IDA_R		0x48
#define HMC5883L_IDB_R		0x34
#define HMC5883L_IDC_R		0x33

enum Mscales
{
	MFS_GAIN0 = 0,
	MFS_GAIN1,
	MFS_GAIN2,
	MFS_GAIN3,
	MFS_GAIN4,
	MFS_GAIN5,
	MFS_GAIN6,
	MFS_GAIN7,
};

// Magnetometer ODR
enum Mrates
{ 
	MRT_0075 = 0,	// 0.75 Hz ODR
	MRT_015,		// 1.5 Hz
	MRT_030,		// 3.0 Hz
	MRT_075,		// 7.5 Hz
	MRT_15,			// 15 Hz default
	MRT_30,			// 30 Hz
	MRT_75,			// 75 Hz ODR    
};

constexpr float getMres(Mscales scale)
{
	// in mG per LSB, with +- of field
	return
		scale == MFS_GAIN0 ?  0.73f : // 0.88 Ga
		scale == MFS_GAIN1 ?  0.92f : // 1.3 Ga default
		scale == MFS_GAIN2 ?  1.22f : // 1.9 Ga
		scale == MFS_GAIN3 ?  1.52f : // 2.5 Ga
		scale == MFS_GAIN4 ?  2.27f : // 4.0 Ga
		scale == MFS_GAIN5 ?  2.56f : // 4.7 Ga
		scale == MFS_GAIN6 ?  3.03f : // 5.6 Ga
		scale == MFS_GAIN7 ?  4.35f : // 8.1 Ga
		0.0f;
}

constexpr Mscales Mscale = MFS_GAIN0;
constexpr Mrates  Mrate  = MRT_75;
constexpr float   mRes   = getMres(Mscale);

int HMC5883L::init()
{
	const uint8_t w = readByte(HMC5883L_ADDRESS, HMC5883L_IDA);
	const uint8_t h = readByte(HMC5883L_ADDRESS, HMC5883L_IDB);
	const uint8_t o = readByte(HMC5883L_ADDRESS, HMC5883L_IDC);

	if (w != HMC5883L_IDA_R || h != HMC5883L_IDB_R || o != HMC5883L_IDC_R)
	{
		return -1;
	}

	// setup device
	writeByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_A,	Mrate  << 2);	// set 1 sample per measurement, ODR, no offset
	writeByte(HMC5883L_ADDRESS, HMC5883L_CONFIG_B,	Mscale << 5);	// set gain, rest must be zeros
	writeByte(HMC5883L_ADDRESS, HMC5883L_MODE,		0x00 );			// no high speed, continuous measurement mode
	
	return 0;
}

int HMC5883L::measure(float &mx, float &my, float &mz)
{
	if (readByte(HMC5883L_ADDRESS, HMC5883L_STATUS) & 0x01) // if status bit RDY is set
	{
		uint8_t rawData[6];

		readBytes(HMC5883L_ADDRESS, HMC5883L_OUT_X_H, 6, &rawData[0]); //read measurement in one pass

		const int16_t sensorOut[3] 
		{
			((int16_t)rawData[0] << 8) | rawData[1], // turn the MSB and LSB into a signed 16-bit value
			((int16_t)rawData[4] << 8) | rawData[5], // registers are xzy (DXRA, DXRB, DZRA, DZRB, DYRA, and DYRB)
			((int16_t)rawData[2] << 8) | rawData[3], // manufacturer even list them in datasheet this way
		};

		// calculate field strength in milliGauss
		mx = (float)sensorOut[0] * mRes;
		my = (float)sensorOut[1] * mRes; 
		mz = (float)sensorOut[2] * mRes; 

		return 0;
	}

	return -1;
}
