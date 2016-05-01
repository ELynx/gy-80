#include "mathhelp.h"

#include <math.h>

bool normalize(float & a, float & b, float & c)
{
	float norm = sqrt(a * a + b * b + c * c);
	
	if (!isgreater(norm, 0.0f))
	{
		return false;
	}
	
	norm = 1.0f / norm;
	
	a = a * norm;
	b = b * norm;
	c = c * norm;
	
	return true;
}

bool normalize(float & a, float & b, float & c, float & d)
{
	float norm = sqrt(a * a + b * b + c * c + d * d);
	
	if (!isgreater(norm, 0.0f))
	{
		return false;
	}
	
	norm = 1.0f / norm;
	
	a = a * norm;
	b = b * norm;
	c = c * norm;
	d = d * norm;
	
	return true;
}
