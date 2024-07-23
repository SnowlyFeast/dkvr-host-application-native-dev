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

		CalibrationManager(TrackerProvider& tk_provider);

		/// <summary>
		/// calling Begin() will cancel the current calibraiton process
		/// </summary>
		void Begin(int index);
		void Continue();
		void Abort();

		int GetCurrentCalibrationTarget() const { return target_index_; }
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
		CalibrationMatrix saved_calibration_;

		std::unique_ptr<std::thread> thread_ptr_;
		std::atomic_bool exit_flag_;

		std::vector<Vector3> gyro_samples_;
		std::vector<Vector3> accel_samples_;
		std::vector<Vector3> mag_samples1_;		// to calibrate mag
		std::vector<Vector3> mag_samples2_;		// to calibrate gyro
		float mag_noise_var_;
		CalibrationMatrix result_calibration_;

		TrackerProvider& tk_provider_;
		Logger& logger_ = Logger::GetInstance();
	};

}	// namespace dkvr