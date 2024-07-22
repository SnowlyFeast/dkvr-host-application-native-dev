#pragma once

#include <vector>

#include "math/matrix.h"

namespace dkvr
{

	class MagCalibrator
	{
	public:
		/// <summary>
		/// Requires samples in a static state
		/// </summary>
		static float CalculateNoiseVariance(const std::vector<Vector3>& samples);


		/// <summary>
		/// Requires samples from multiple directions
		/// </summary>
		static Matrix CalculateCalibrationMatrix(const std::vector<Vector3>& samples, float noise_variance);

	};

}	// namespace dkvr