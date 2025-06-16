#pragma once

#include <cmath>

#ifndef normalizeAngle
#define normalizeAngle(val) (std::atan2(std::sin(val),std::cos(val)))
#else
#warning normalizeAngle was previously defined
#endif
