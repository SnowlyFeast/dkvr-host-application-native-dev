#pragma once

#include <vector>

#include "math/vector.h"
#include "tracker/tracker_imu.h"

namespace dkvr
{

	class GyroCalibrator
	{
	public:
		static Vector3 CalculateGyroOffset(const std::vector<IMUReadings>& samples)
		{
			Vector3 mean{};
			for (const IMUReadings& s : samples)
				mean += s.gyr;
			mean /= samples.size();

			return mean;
		}
	};

}	// namespace dkvr