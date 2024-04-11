#include "calibrator/gyro_calibrator.h"

namespace dkvr
{

	Vector3 GyroCalibrator::CalculateOffset(const std::vector<IMUReadings>& samples)
	{
		Vector3 mean{};
		for (const IMUReadings& s : samples)
			mean += s.gyr;
		mean /= samples.size();

		return mean;
	}

}	// namespace dkvr