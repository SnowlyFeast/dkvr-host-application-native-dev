#pragma once

#include <vector>

#include "math/matrix.h"
#include "tracker/tracker_imu.h"

namespace dkvr
{

	class CommonCalibrator
	{
	public:
		static void CalibrateSamples(std::vector<Vector3>& samples, const Matrix& calibration_matrix);
		static Vector3 CalculateNoiseVariance(const std::vector<Vector3>& samples, const Matrix& calibration_matrix);
	};

}	// namespace dkvr