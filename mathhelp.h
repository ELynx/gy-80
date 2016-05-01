#ifndef mathhelp_h_
#define mathhelp_h_

#include <Arduino.h>

constexpr float deg2rad(float deg) { return deg * (float)DEG_TO_RAD; }
constexpr float rad2deg(float rad) { return rad * (float)RAD_TO_DEG; }

static_assert(123.0f == deg2rad(rad2deg(123.0f)), "Float error is too big");

bool normalize(float & a, float & b, float & c);
bool normalize(float & a, float & b, float & c, float & d);

#endif
