#pragma once

#include <atomic>
#include <vector>

#include "Eigen/Core"

#include "calibrator/type.h"
#include "tracker/tracker_data.h"


namespace dkvr
{
	/// <summary>
	/// <para>Just simple interface for consistent-looking calibrator.</para>
	/// <para>Nothing more.</para>
	/// </summary>
	class Calibrator
	{
	public:
		virtual void Reset() = 0;
		virtual void Accumulate(SampleType type, const std::vector<RawDataSet>& samples) = 0;
		virtual void Calculate() = 0;

		virtual CalibrationMatrix GetCalibrationMatrix() = 0;
		virtual Eigen::Vector3f GetNoiseVairance() = 0;

		int GetProgress() const { return progress_perc_.load(); }

	protected:
		std::atomic_int progress_perc_ = 0;
	};

}	// namespace dkvr