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

		bool IsCalculationFinished() const { return !calculating_; }

		void Reset() override;
		void Accumulate(SampleType type, const std::vector<RawDataSet>& samples) override;
		void Calculate() override;
		void SetTimeStep(float time_step) { time_step_ = time_step; }
		void SetMagCalibrationMatrix(CalibrationMatrix calib) { mag_calib_ = calib; }

		CalibrationMatrix GetCalibrationMatrix() override { return result_; }
		Eigen::Vector3f GetNoiseVairance() override { return noise_var_; }


	private:
		struct SampleTuple
		{
			Eigen::Vector3f gyro, old_mag, new_mag;
		};

		void RunGradientDescent();
		void AddGradient(const SampleTuple& tuple);

		float time_step_;
		int iteration_;
		int batch_size_;
		float learn_rate_;

		std::vector<RawDataSet> uncalibrated_set_;
		std::vector<SampleTuple> samples_;
		Eigen::Matrix<float, 1, 12> gradient_;

		CalibrationMatrix mag_calib_;
		CalibrationMatrix result_;
		Eigen::Vector3f noise_var_;

		std::atomic_bool calculating_ = false;
	};

}	// namespace dkvr