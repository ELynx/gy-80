#include "ADXL345.h"

#include "i2chelp.h"

#define WHO_AM_I_ADXL345		0x00 // question for who am i
#define I_AM_ADXL345			0xE5 // response for who am i
#define ADXL345_THRESH_TAP		0x1D // tap threshold
#define ADXL345_OFSX			0x1E // x-axis offset
#define ADXL345_OFSY			0x1F // y-axis offset
#define ADXL345_OFSZ			0x20 // z-axis offset
#define ADXL345_DUR				0x21 // tap duration
#define ADXL345_LATENT			0x22 // tap latency
#define ADXL345_WINDOW			0x23 // tap window
#define ADXL345_THRESH_ACT		0x24 // activity threshold
#define ADXL345_THRESH_INACT	0x25 // inactivity threshold
#define ADXL345_TIME_INACT		0x26 // inactivity time
#define ADXL345_ACT_INACT_CTL	0x27 // axis enable control for activity/inactivity detection
#define ADXL345_THRESH_FF		0x28 // free-fall threshold
#define ADXL345_TIME_FF			0x29 // free-fall time
#define ADXL345_TAP_AXES		0x2A // axis control for single/double tap
#define ADXL345_ACT_TAP_STATUS	0x2B // source of single/double tap
#define ADXL345_BW_RATE			0x2C // data rate and power mode control
#define ADXL345_POWER_CTL		0x2D // power-saving features control
#define ADXL345_INT_ENABLE		0x2E // interrupt enable control
#define ADXL345_INT_MAP			0x2F // interrupt mapping control
#define ADXL345_INT_SOURCE		0x30 // source of interrupts
#define ADXL345_DATA_FORMAT		0x31 // data format control
#define ADXL345_DATAX0			0x32 // z-axis data 0
#define ADXL345_DATAX1			0x33 // z-axis data 1
#define ADXL345_DATAY0			0x34 // y-axis data 0
#define ADXL345_DATAY1			0x35 // y-axis data 1
#define ADXL345_DATAZ0			0x36 // z-axis data 0
#define ADXL345_DATAZ1			0x37 // z-axis data 1
#define ADXL345_FIFO_CTL		0x38 // FIFO control
#define ADXL345_FIFO_STATUS		0x39 // FIFO status
#define ADXL345_ADDRESS			0x53 // device address when ADO = 0

// accelerometer measure limits
enum Ascales
{
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_16G
};

// accelerometer ODR and Bandwidth
enum Arates
{
	ARTBW_010_005 = 0,	// 0.1 Hz ODR, 0.05Hz bandwidth
	ARTBW_020_010,
	ARTBW_039_020,
	ARTBW_078_039,
	ARTBW_156_078,
	ARTBW_313_156,
	ARTBW_125_625,
	ARTBW_25_125,
	ARTBW_50_25,
	ARTBW_100_50,
	ARTBW_200_100,
	ARTBW_400_200,
	ARTBW_800_400,
	ARTBW_1600_800,
	ARTBW_3200_1600		// 3200 Hz ODR, 1600 Hz bandwidth
};

constexpr float getAres(Ascales scale)
{
	// possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs (11).
	return
		scale == AFS_2G  ?  2.0f/( 512.0f*64.0f) : // 10-bit 2s-complement
		scale == AFS_4G  ?  4.0f/(1024.0f*32.0f) : // 11-bit 2s-complement
		scale == AFS_8G  ?  8.0f/(2048.0f*16.0f) : // 12-bit 2s-complement
		scale == AFS_16G ? 16.0f/(4096.0f* 8.0f) : // 13-bit 2s-complement
		0.0f;
}

constexpr Ascales Ascale = AFS_4G;
constexpr Arates  Arate  = ARTBW_100_50;
constexpr float   aRes   = getAres(Ascale);

int ADXL345::init()
{
	const auto who = readByte(ADXL345_ADDRESS, WHO_AM_I_ADXL345);

	if (who != I_AM_ADXL345)
	{
		return -1;
	}

	// preset device state
	writeByte(ADXL345_ADDRESS, ADXL345_POWER_CTL,	0x00);			// put device in standby mode
	delay(12); // worst case 11.1 ms from datasheet

	// setup device
	writeByte(ADXL345_ADDRESS, ADXL345_BW_RATE,		Arate);			// normal power operation, ODR, bandwidth
	writeByte(ADXL345_ADDRESS, ADXL345_DATA_FORMAT,	0x04 | Ascale);	// set full scale range left justify MSB
	writeByte(ADXL345_ADDRESS, ADXL345_FIFO_CTL,	0x00);			// bypass FIFO
	
	// start measurment process
	writeByte(ADXL345_ADDRESS, ADXL345_POWER_CTL,	0x08);			// put device in normal mode
	delay(12); // settle again

	return 0;
}

int ADXL345::measure(float &ax, float &ay, float &az)
{
	if (readByte(ADXL345_ADDRESS, ADXL345_INT_SOURCE) & 0x80) // when data ready bit is high
	{
		uint8_t rawData[6];

		readBytes(ADXL345_ADDRESS, ADXL345_DATAX0, 6, &rawData[0]); //read measurement in one pass

		const int16_t sensorOut[3] 
		{
			((int16_t)rawData[1] << 8) | rawData[0], // turn the MSB and LSB into a signed 16-bit value
			((int16_t)rawData[3] << 8) | rawData[2],
			((int16_t)rawData[5] << 8) | rawData[4],
		};

		// calculate the accleration value in Gs
		ax = (float)sensorOut[0] * aRes;
		ay = (float)sensorOut[1] * aRes;
		az = (float)sensorOut[2] * aRes;

		return 0;
	}

	return -1;
}
