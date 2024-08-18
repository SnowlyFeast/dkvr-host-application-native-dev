#pragma once

#include <vector>

#include "Eigen/Core"

#include "calibrator/calibrator.h"
#include "calibrator/type.h"
#include "tracker/tracker_data.h"

namespace dkvr
{

	class MagCalibrator : public Calibrator
	{
	public:
		MagCalibrator() : result_(), noise_var_() { }

		void Reset() override { }
		void Accumulate(SampleType type, const std::vector<RawDataSet>& samples) override;
		void Calculate() override { }

		CalibrationMatrix GetCalibrationMatrix() override { return result_; }
		Eigen::Vector3f GetNoiseVairance() override { return noise_var_; }

	private:
		CalibrationMatrix result_;
		Eigen::Vector3f noise_var_;
	};

}	// namespace dkvr