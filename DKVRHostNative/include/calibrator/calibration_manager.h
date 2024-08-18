#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "Eigen/Dense"

#include "calibrator/accel_calibrator.h"
#include "calibrator/gyro_calibrator.h"
#include "calibrator/mag_calibrator.h"
#include "calibrator/type.h"

#include "tracker/tracker.h"
#include "tracker/tracker_data.h"
#include "tracker/tracker_provider.h"

#include "util/logger.h"

namespace dkvr
{

	class CalibrationManager
	{
	public:
		enum class CalibrationStatus
		{
			Idle,
			Configuring,
			StandBy,
			Recording,
			Calibrating
		};

		CalibrationManager(TrackerProvider& tk_provider);

		/// <summary>
		/// calling Begin() will cancel the current calibraiton process
		/// </summary>
		void Begin(int index);
		void Continue();
		void Abort();

		int GetCurrentCalibrationTarget() const { return target_index_; }

		CalibrationStatus GetStatus() const { return status_; }
		SampleType GetRequiredSampleType() const { return sample_type_; }

		std::string GetStatusAsString() const;
		std::string GetRequiredSampleTypeAsString() const;

	private:
		void Reset();

		void ConfiguringThreadLoop();
		void RecordingThreadLoop();
		void HandleSamples();
		void ApplyCalibration();

		GyroCalibrator gyro_calibrator_;
		AccelCalibrator accel_calibrator_;
		MagCalibrator mag_calibrator_;

		std::unique_ptr<std::thread> thread_ptr_;
		std::atomic_bool exit_flag_;
		
		// calibration manager status
		CalibrationStatus status_;
		SampleType sample_type_;

		// tracker
		int target_index_;
		uint8_t saved_behavior_;
		TrackerCalibration saved_calibration_;
		TrackerCalibration result_calibration_;

		std::vector<RawDataSet> samples_;

		TrackerProvider& tk_provider_;
		Logger& logger_ = Logger::GetInstance();
	};

}	// namespace dkvr