#include "calibrator/calibration_manager.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include "Eigen/Dense"

#include "calibrator/common_calibrator.h"

#include "tracker/tracker.h"
#include "tracker/tracker_configuration.h"

namespace dkvr
{

	namespace 
	{
		constexpr size_t kRequiredStaticSampleSize = 100;
		constexpr size_t kRequiredRotationalSampleSize = 1000;
		constexpr std::chrono::milliseconds kValidationInterval(500);

		const std::string kStringIdle = "Idle";
		const std::string kStringConfiguring = "Configuring";
		const std::string kStringStandBy = "StandBy";
		const std::string kStringRecording = "Recording";
		const std::string kStringCalibrating = "Calibrating";

		const std::string kStringXPositive = "Place the tracker with Positive X-axis facing up.";
		const std::string kStringXNegative = "Place the tracker with Negative X-axis facing up.";
		const std::string kStringYPositive = "Place the tracker with Positive Y-axis facing up.";
		const std::string kStringYNegative = "Place the tracker with Negative Y-axis facing up.";
		const std::string kStringZPositive = "Place the tracker with Positive Z-axis facing up.";
		const std::string kStringZNegative = "Place the tracker with Negative Z-axis facing up.";
		const std::string kStringRotational = "Slowly rotate the tracker.";

		bool IsStaticConstraintSatisfied(const RawDataSet& current, const RawDataSet& prev)
		{
			constexpr float linear_accel_limit = 0.2f;

			Eigen::Vector3f v_cur{ current.acc[0], current.acc[1], current.acc[2] };
			Eigen::Vector3f v_prev{ prev.acc[0], prev.acc[1], prev.acc[2] };
			Eigen::Vector3f diff = v_cur - v_prev;

			return diff.norm() < linear_accel_limit;
		}
	}

	CalibrationManager::CalibrationManager(TrackerProvider& tk_provider) :
		gyro_calibrator_(0.01f), 
		accel_calibrator_(), 
		mag_calibrator_(), 
		status_(CalibrationStatus::StandBy), 
		sample_type_(SampleType::ZNegative), 
		target_index_(-1), 
		saved_behavior_(TrackerBehavior::kBitmaskInvalid), 
		saved_calibration_{}, 
		thread_ptr_(nullptr), 
		exit_flag_(false), 
		samples_(),
		result_calibration_{}, 
		tk_provider_(tk_provider)
	{
		samples_.reserve(kRequiredRotationalSampleSize);
	}

	void CalibrationManager::Begin(int index)
	{
		Abort();
		
		AtomicTracker target = tk_provider_.FindByIndex(index);
		if (target)
		{
			target_index_ = index;
			thread_ptr_ = std::make_unique<std::thread>(&CalibrationManager::ConfiguringThreadLoop, this);
			logger_.Info("Begin calibration of {}.", target->name());
		}
	}

	void CalibrationManager::Continue()
	{
		if (status_ != CalibrationStatus::StandBy)
			return;

		if (thread_ptr_)
		{
			if (thread_ptr_->joinable())	// actually not gonna happen
				thread_ptr_->join();
			thread_ptr_.reset();
		}
		
		status_ = CalibratorStatus::Recording;
		progress_perc_ = 0;
		thread_ptr_ = std::make_unique<std::thread>(&CalibrationManager::RecordingThreadLoop, this);
	}

	void CalibrationManager::Abort()
	{
		if (status_ != CalibrationStatus::StandBy)
		{
			// stop thread
			exit_flag_ = true;
			if (thread_ptr_)
			{
				if (thread_ptr_->joinable())
					thread_ptr_->join();
				thread_ptr_.reset();
			}

			// rollback tracker config
			if (target_index_ != -1 && saved_behavior_ != TrackerBehavior::kBitmaskInvalid)
			{
				AtomicTracker target = tk_provider_.FindByIndex(target_index_);
				target->set_behavior(saved_behavior_);
				target->set_calibration(saved_calibration_);
			}

			logger_.Info("Calibration process aborted.");
		}

		Reset();
	}

	std::string CalibrationManager::GetStatusAsString() const
	{
		switch (status_)
		{
		case CalibrationStatus::Idle:
			return kStringIdle;

		case CalibrationStatus::Configuring:
			return kStringConfiguring;

		case CalibrationStatus::StandBy:
			return kStringStandBy;

		case CalibrationStatus::Recording:
			return kStringRecording;

		case CalibrationStatus::Calibrating:
			return kStringCalibrating;

		default:
			break;
		}

		return "";
	}

	std::string CalibrationManager::GetRequiredSampleTypeAsString() const
	{
		switch (sample_type_)
		{
		case SampleType::ZNegative:
			return kStringZNegative;

		case SampleType::ZPositive:
			return kStringZPositive;

		case SampleType::YNegative:
			return kStringYNegative;

		case SampleType::YPositive:
			return kStringYPositive;

		case SampleType::XNegative:
			return kStringXNegative;

		case SampleType::XPositive:
			return kStringXPositive;

		case SampleType::Rotational:
			return kStringRotational;

		default:
			break;
		}
		return "";
	}

	void CalibrationManager::Reset()
	{
		gyro_calibrator_.Reset();
		accel_calibrator_.Reset();
		mag_calibrator_.Reset();

		status_ = CalibrationStatus::StandBy;
		sample_type_ = SampleType(0);

		exit_flag_ = false;

		target_index_ = -1;
		saved_behavior_ = TrackerBehavior::kBitmaskInvalid;
		saved_calibration_.Reset();
		result_calibration_.Reset();

		samples_.clear();
	}

	void CalibrationManager::ConfiguringThreadLoop()
	{
		constexpr TrackerBehavior behavior{ .led = true, .active = true, .raw = true, .nominal = false };
		constexpr TrackerCalibration calibration = {
			// column-major
			.gyr_transform = { 1, 0, 0,   0, 1, 0,   0, 0, 1,   0, 0, 0 },
			.acc_transform = { 1, 0, 0,   0, 1, 0,   0, 0, 1,   0, 0, 0 },
			.mag_transform = { 1, 0, 0,   0, 1, 0,   0, 0, 1,   0, 0, 0 },
			.gyr_noise_var = { 0, 0, 0 },
			.acc_noise_var = { 0, 0, 0 },
			.mag_noise_var = { 0, 0, 0 }
		};

		status_ = CalibrationStatus::Configuring;

		// configure target
		{
			AtomicTracker target = tk_provider_.FindByIndex(target_index_);
			saved_behavior_ = target->behavior();
			saved_calibration_ = target->calibration();

			target->set_behavior(behavior);
			target->set_calibration(calibration);
		}	// must release the tracker to update it's status

		while (!exit_flag_)
		{
			{
				AtomicTracker target = tk_provider_.FindByIndex(target_index_);
				if (target->IsAllValid())
					break;
			}
			// wait for validation
			std::this_thread::sleep_for(kValidationInterval);
		}

		status_ = CalibrationStatus::StandBy;
		logger_.Debug("[Calibration] (Step1/8) Tracker configured.");
	}

	void CalibrationManager::RecordingThreadLoop()
	{
		// begin sample record
		samples_.clear();
		while (!exit_flag_)
		{
			// get IMU readings if updated
			bool yield = false;
			RawDataSet data;
			{
				AtomicTracker target = tk_provider_.FindByIndex(target_index_);
				if (target->IsRawDataUpdated())
					data = target->raw_data();
				else
					yield = true;
			}	// tracker should be released before yielding thread

			if (yield)
			{
				std::this_thread::yield();
				continue;
			}

			// handle by sample type
			if (sample_type_ == SampleType::Rotational)
			{
				samples_.push_back(data);
				if (samples_.size() >= kRequiredRotationalSampleSize)
					break;
			}
			else
			{
				if (samples_.empty() || IsStaticConstraintSatisfied(data, samples_.back()))
				{
					samples_.push_back(data);
					if (samples_.size() >= kRequiredStaticSampleSize)
						break;
				}
			}
		}

		// aborted
		if (exit_flag_)	
			return;

		HandleSamples();

		// step ended
		if (sample_type_ != SampleType::Rotational)
		{
			// record next samples
			sample_type_ = SampleType(static_cast<int>(sample_type_) + 1);
			status_ = CalibrationStatus::StandBy;
			logger_.Debug("[Calibration] (Step{}/8) Sample gathered.", static_cast<int>(sample_type_) + 1);
		}
		else
		{
			// recording finished
			status_ = CalibrationStatus::Calibrating;
			logger_.Debug("[Calibration] (Step8/8) Calibrating...");
			ApplyCalibration();
		}
	}

	void CalibrationManager::HandleSamples()
	{
		gyro_calibrator_.Accumulate(sample_type_, samples_);
		accel_calibrator_.Accumulate(sample_type_, samples_);
		mag_calibrator_.Accumulate(sample_type_, samples_);
	}

	void CalibrationManager::ApplyCalibration()
	{
		gyro_calibrator_.Calculate();
		accel_calibrator_.Calculate();
		mag_calibrator_.Calculate();

		gyro_calibrator_.GetCalibrationMatrix().CopyTo(result_calibration_.gyr_transform);
		accel_calibrator_.GetCalibrationMatrix().CopyTo(result_calibration_.acc_transform);
		mag_calibrator_.GetCalibrationMatrix().CopyTo(result_calibration_.mag_transform);

		Eigen::Vector3f gyr_noise_var = gyro_calibrator_.GetNoiseVairance();
		Eigen::Vector3f acc_noise_var = accel_calibrator_.GetNoiseVairance();
		Eigen::Vector3f mag_noise_var = mag_calibrator_.GetNoiseVairance();
		std::copy_n(gyr_noise_var.data(), 3, result_calibration_.gyr_noise_var);
		std::copy_n(acc_noise_var.data(), 3, result_calibration_.acc_noise_var);
		std::copy_n(mag_noise_var.data(), 3, result_calibration_.mag_noise_var);

		{
			AtomicTracker target = tk_provider_.FindByIndex(target_index_);
			target->set_behavior(saved_behavior_);
			target->set_calibration(result_calibration_);
		}

		while (!exit_flag_)
		{
			{
				AtomicTracker target = tk_provider_.FindByIndex(target_index_);
				if (target->IsAllValid())
					break;
			}
			// wait for validation
			std::this_thread::sleep_for(kValidationInterval);
		}

		// aborted
		if (exit_flag_)
			return;

		logger_.Debug("[Calibration] (9/9) Calibration finished.");
		logger_.Info("Calibration finished.");
		Reset();
	}

}	// namespace dkvr