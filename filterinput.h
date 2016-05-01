#ifndef filterinput_h_
#define filterinput_h_

#include "imudata.h"

struct FilterInput : ImuData
{
	float deltaT;
};

#endif
