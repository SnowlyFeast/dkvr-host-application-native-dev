#pragma once

#include <vector>

#include "math/matrix.h"
#include "tracker/tracker_imu.h"

namespace dkvr
{

	class MagCalibrator
	{
	public:
		static Matrix CalculateCalibrationMatrix(const std::vector<IMUReadings>& samples);
	};

}	// namespace dkvr