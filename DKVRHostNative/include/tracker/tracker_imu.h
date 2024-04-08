#pragma once

#include "math/quaternion.h"
#include "math/vector.h"

namespace dkvr {

	struct IMUReadings
	{
		Quaternion quat;
		Vector3 gyr, acc, mag;
	};

}	// namespace dkvr