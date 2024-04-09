#pragma once

#include <vector>

#include "math/vector.h"
#include "tracker/tracker_imu.h"

namespace dkvr
{

	class GyroCalibrator
	{
	public:
		static Vector3 CalculateOffset(const std::vector<IMUReadings>& samples);
	};

}	// namespace dkvr