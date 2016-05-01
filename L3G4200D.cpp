#include "L3G4200D.h"

#include "i2chelp.h"
#include "mathhelp.h"

#define WHO_AM_I_L3G4200D		0x0F
#define I_AM_L3G4200D			0xD3
#define L3G4200D_CTRL_REG1		0x20
#define L3G4200D_CTRL_REG2		0x21
#define L3G4200D_CTRL_REG3		0x22
#define L3G4200D_CTRL_REG4		0x23
#define L3G4200D_CTRL_REG5		0x24
#define L3G4200D_REFERENCE		0x25
#define L3G4200D_OUT_TEMP		0x26
#define L3G4200D_STATUS_REG		0x27
#define L3G4200D_OUT_X_L		0x28
#define L3G4200D_OUT_X_H		0x29
#define L3G4200D_OUT_Y_L		0x2A
#define L3G4200D_OUT_Y_H		0x2B
#define L3G4200D_OUT_Z_L		0x2C
#define L3G4200D_OUT_Z_H		0x2D
#define L3G4200D_FIFO_CTRL_REG	0x2E
#define L3G4200D_FIFO_SRC_REG	0x2F
#define L3G4200D_INT1_CFG		0x30
#define L3G4200D_INT1_SRC		0x31
#define L3G4200D_INT1_TSH_XH	0x32
#define L3G4200D_INT1_TSH_XL	0x33
#define L3G4200D_INT1_TSH_YH	0x34
#define L3G4200D_INT1_TSH_YL	0x35
#define L3G4200D_INT1_TSH_ZH	0x36
#define L3G4200D_INT1_TSH_ZL	0x37
#define L3G4200D_INT1_DURATION	0x38
#define L3G4200D_ADDRESS		0x69 // device address when ADO = 0

// gyro measure limits
enum Gscales
{
	GFS_250DPS = 0,
	GFS_500DPS,
	GFS_2000DPS
};

// gyro ODR and Bandwidth with 4 bits
enum Grates
{
	GRTBW_100_125 = 0,	// 100 Hz ODR, 12.5 Hz bandwidth
	GRTBW_100_25,
	GRTBW_100_25a,
	GRTBW_100_25b,
	GRTBW_200_125,
	GRTBW_200_25,
	GRTBW_200_50,
	GRTBW_200_70,
	GRTBW_400_20,
	GRTBW_400_25,
	GRTBW_400_50,
	GRTBW_400_110,
	GRTBW_800_30,
	GRTBW_800_35,
	GRTBW_800_50,
	GRTBW_800_110		// 800 Hz ODR, 110 Hz bandwidth   
};

constexpr float getGres(Gscales scale)
{
	// possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), and 2000 DPS (10 or 11). 
	return
		scale == GFS_250DPS  ?  250.0f/32768.0f :
		scale == GFS_500DPS  ?  500.0f/32768.0f :
		scale == GFS_2000DPS ? 2000.0f/32768.0f :
		0.0f;
}

constexpr Gscales Gscale = GFS_500DPS;
constexpr Grates  Grate  = GRTBW_100_25;
constexpr float   gRes   = deg2rad(getGres(Gscale));

int L3G4200D::init()
{
	const auto who = readByte(L3G4200D_ADDRESS, WHO_AM_I_L3G4200D);

	if (who != I_AM_L3G4200D)
	{
		return -1;
	}

	writeByte(L3G4200D_ADDRESS, L3G4200D_CTRL_REG1,	Grate << 4 | 0x0F);	// set gyro ODR and bandwidth, normal mode, all axis active
	//skip register 2, has something to do with calibration
	//skip register 3, don`t use interrupts
	writeByte(L3G4200D_ADDRESS, L3G4200D_CTRL_REG4,	Gscale << 4);		// set cont. update, gyro scale, no self-test
	writeByte(L3G4200D_ADDRESS, L3G4200D_CTRL_REG5,	0x00);				// disable FIFO

	return 0;
}

int L3G4200D::measure(float &gx, float &gy, float &gz)
{
	if (readByte(L3G4200D_ADDRESS, L3G4200D_STATUS_REG) & 0x08) // when zyxda bit is high
	{
		uint8_t rawData[6];

		readBytes(L3G4200D_ADDRESS, L3G4200D_OUT_X_L | 0x80, 6, &rawData[0]); //read measurement in one pass

		const int16_t sensorOut[3] 
		{
			((int16_t)rawData[1] << 8) | rawData[0], // turn the MSB and LSB into a signed 16-bit value
			((int16_t)rawData[3] << 8) | rawData[2],
			((int16_t)rawData[5] << 8) | rawData[4],
		};

		// calculate the angle rate in radians per second
		gx = (float)sensorOut[0] * gRes;
		gy = (float)sensorOut[1] * gRes;   
		gz = (float)sensorOut[2] * gRes;  

		return 0;
	}

	return -1;
}
