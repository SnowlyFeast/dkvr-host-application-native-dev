#pragma once

#include <vector>

#include "math/vector.h"
#include "math/matrix.h"

namespace dkvr
{

	class GyroCalibrator
	{
	public:
		GyroCalibrator(int iteration = 100, int batch_size = 30, float learn_rate = 0.01f) :
			iteration_(iteration), batch_size_(batch_size), learn_rate_(learn_rate), calibration_matrix_(3, 4), gradient_(12) {}

		Matrix CalculateCalibrationMatrix(const std::vector<Vector3>& gyro_samples, const std::vector<Vector3>& mag_samples, float time_step);

	private:
		struct SampleTuple
		{
			Vector3 gyro, old_mag, new_mag;
		};

		void AddGradient(const Vector3& gyro, const Vector3& old_mag, const Vector3& new_mag, float time_step);

		int iteration_;
		int batch_size_;
		float learn_rate_;

		Matrix calibration_matrix_;
		Vector gradient_;
	};

}	// namespace dkvr