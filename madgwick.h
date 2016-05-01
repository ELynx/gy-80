#ifndef madgwick_h_
#define madgwick_h_

#include "quart.h"
#include "filterinput.h"

void MadgwickQuaternionUpdate(Quart& quart, FilterInput input);

#endif
