#pragma once

#include <vector>

#include "Eigen/Core"

#include "calibrator/calibrator.h"
#include "calibrator/type.h"
#include "tracker/tracker_data.h"

namespace dkvr
{

	class GyroCalibrator : public Calibrator
	{
	public:
		GyroCalibrator(float time_step, int iteration = 100, int batch_size = 30, float learn_rate = 0.01f) :
			time_step_(time_step),
			iteration_(iteration),
			batch_size_(batch_size),
			learn_rate_(learn_rate),
			gradient_(),
			result_(),
			noise_var_()
		{ }

		void Reset() override;
		void Accumulate(SampleType type, const std::vector<RawDataSet>& samples) override;
		void Calculate() override { }
		void SetTimeStep(float time_step) { time_step_ = time_step; }

		CalibrationMatrix GetCalibrationMatrix() override { return result_; }
		Eigen::Vector3f GetNoiseVairance() override { return noise_var_; }


	private:
		struct SampleTuple
		{
			SampleTuple(const Vector3f& gyr, const Vector3f& old_mag, const Vector3f& new_mag) :
				gyro{ gyr[0], gyr[1], gyr[2] },
				old_mag{ old_mag[0], old_mag[1], old_mag[2] },
				new_mag{ new_mag[0], new_mag[1], new_mag[2] } { }

			Eigen::Vector3f gyro, old_mag, new_mag;
		};

		void RunGradientDescent(const std::vector<RawDataSet>& samples);
		void AddGradient(const SampleTuple& tuple);

		float time_step_;
		int iteration_;
		int batch_size_;
		float learn_rate_;

		Eigen::Matrix<float, 1, 12, Eigen::RowMajor> gradient_;

		CalibrationMatrix result_;
		Eigen::Vector3f noise_var_;
	};

}	// namespace dkvr