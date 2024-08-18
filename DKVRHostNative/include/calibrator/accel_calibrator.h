#pragma once

#include <vector>

#include "Eigen/Core"

#include "calibrator/calibrator.h"
#include "calibrator/type.h"
#include "tracker/tracker_data.h"

namespace dkvr
{
	
	class AccelCalibrator : public Calibrator
	{
	public:
		AccelCalibrator() : accumulated_{}, samples_avg_{}, result_(), noise_var_() {};

		void Reset() override;
		void Accumulate(SampleType type, const std::vector<RawDataSet>& samples) override;
		void Calculate() override;

		CalibrationMatrix GetCalibrationMatrix() override { return result_; }
		Eigen::Vector3f GetNoiseVairance() override { return noise_var_; }

	private:
		bool accumulated_[6];
		Eigen::Vector3f samples_avg_[6];
		CalibrationMatrix result_;
		Eigen::Vector3f noise_var_;
	};

}	// namespace dkvr