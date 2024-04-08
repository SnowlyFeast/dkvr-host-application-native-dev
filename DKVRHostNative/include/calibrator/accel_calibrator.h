#pragma once

#include <vector>

#include "math/matrix.h"
#include "math/vector.h"
#include "tracker/tracker_imu.h"

namespace dkvr
{
	
	class AccelCalibrator
	{
	public:
		enum class Axis
		{
			XPositive,
			XNegative,
			YPositive,
			YNegative,
			ZPositive,
			ZNegative
		};
		
		AccelCalibrator() : accumulated_{}, samples_avg_{} {};

		void Reset();
		void AccumulateSample(Axis axis, const std::vector<IMUReadings>& samples);
		Matrix CalculateCalibrationMatrix();

	private:
		bool accumulated_[6];
		Vector3 samples_avg_[6];
	};

}	// namespace dkvr