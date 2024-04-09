#pragma once

#include <vector>

#include "math/matrix.h"
#include "tracker/tracker_imu.h"

namespace dkvr
{

	class MagCalibrator
	{
	public:
		/// <summary>
		/// Requires samples in a static state
		/// </summary>
		static float CalculateNoiseVariance(const std::vector<IMUReadings>& samples);


		/// <summary>
		/// Requires samples from multiple directions
		/// </summary>
		static Matrix CalculateCalibrationMatrix(const std::vector<IMUReadings>& samples, float noise_variance);
	};

}	// namespace dkvr