#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "calibrator/accel_calibrator.h"
#include "calibrator/gyro_calibrator.h"
#include "calibrator/mag_calibrator.h"

#include "math/vector.h"
#include "math/matrix.h"

#include "tracker/tracker.h"
#include "tracker/tracker_imu.h"
#include "tracker/tracker_provider.h"

#include "util/logger.h"

namespace dkvr
{

	class CalibrationManager
	{
	public:
		enum class CalibrationStatus
		{
			StandBy,
			ConfiguringTracker,
			WaitingContinue,
			RecordingIMU,
			Calibrating
		};

		enum class SampleTypes : int
		{
			NegativeZ,
			PositiveZ,
			NegativeY,
			PositiveY,
			NegativeX,
			PositiveX,
			Rotational
		};

		CalibrationManager();

		/// <summary>
		/// calling Begin() will cancel the current calibraiton process
		/// </summary>
		void Begin(const Tracker* target);
		void Continue();
		void Abort();

		CalibrationStatus GetStatus() const;
		SampleTypes GetRequiredSampleType() const;

		std::string GetStatusAsString() const;
		std::string GetRequiredSampleTypeAsString() const;

	private:
		void Reset();

		void ConfiguringThreadLoop();
		void RecordingThreadLoop();
		void HandleSamples();
		void ApplyCalibration();

		GyroCalibrator gyro_calib_;
		AccelCalibrator accel_calib_;
		MagCalibrator mag_calib_;
		
		CalibrationStatus status_;
		SampleTypes sample_type_;
		int target_index_;
		uint8_t saved_behavior_;

		std::unique_ptr<std::thread> thread_ptr_;
		std::atomic_bool exit_flag_;

		std::vector<IMUReadings> samples_;
		size_t required_size_;
		float mag_noise_var_;
		Calibration result_;

		TrackerProvider& tk_provider_ = TrackerProvider::GetInstance();
		Logger& logger_ = Logger::GetInstance();
	};

}	// namespace dkvr