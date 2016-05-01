#ifndef imudata_h_
#define imudata_h_

struct ImuData
{
	float & ax() { return values[0]; }
	float & ay() { return values[1]; }
	float & az() { return values[2]; }

	float & gx() { return values[3]; }
	float & gy() { return values[4]; }
	float & gz() { return values[5]; }	
	
	float & mx() { return values[6]; }
	float & my() { return values[7]; }
	float & mz() { return values[8]; }

	float values[9];
};

#endif
