#pragma once

#include <vector>

#include "Eigen/Core"

#include "calibrator/type.h"

namespace dkvr
{

	class CommonCalibrator
	{
	public:
		static Eigen::Vector3f CalculateNoiseVariance(const std::vector<Eigen::Vector3f>& samples);
		static void TransformNoiseVariance(Eigen::Vector3f& noise_var, const CalibrationMatrix& calib);
	};

}	// namespace dkvr