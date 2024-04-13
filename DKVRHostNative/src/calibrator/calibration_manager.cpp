#include "calibrator/calibration_manager.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include "tracker/tracker.h"
#include "tracker/tracker_configuration.h"

#define GLUE(x, y)			(x ## y)
#define CONST_STRING(name)	GLUE(kString, name)

namespace dkvr
{

	constexpr size_t kRequiredStaticSampleSize = 100;
	constexpr size_t kRequiredRotationalSampleSize = 300;
	constexpr std::chrono::milliseconds kValidationInterval(500);
	constexpr std::chrono::milliseconds kRecordingInterval(50);

	static const std::string kStringStandBy = "Stand By";
	static const std::string kStringConfiguringTracker = "Configuring Tracker";
	static const std::string kStringWaitingContinue = "Waiting Continue";
	static const std::string kStringRecordingIMU = "Recording IMU";
	static const std::string kStringCalibrating = "Calibrating";

	static const std::string kStringPositiveX = "Place the tracker with Positive X-axis facing up.";
	static const std::string kStringNegativeX = "Place the tracker with Negative X-axis facing up.";
	static const std::string kStringPositiveY = "Place the tracker with Positive Y-axis facing up.";
	static const std::string kStringNegativeY = "Place the tracker with Negative Y-axis facing up.";
	static const std::string kStringPositiveZ = "Place the tracker with Positive Z-axis facing up.";
	static const std::string kStringNegativeZ = "Place the tracker with Negative Z-axis facing up.";
	static const std::string kStringRotational = "Slowly rotate the tracker.";

	CalibrationManager::CalibrationManager(TrackerProvider& tk_provider) :
		gyro_calib_(), accel_calib_(), mag_calib_(), status_(CalibrationStatus::StandBy),
		sample_type_(SampleTypes::NegativeZ), target_index_(-1), saved_behavior_(TrackerBehavior::kInvalid), thread_ptr_(nullptr), exit_flag_(false),
		samples_(), required_size_(0), mag_noise_var_(0.0f), result_{}, tk_provider_(tk_provider)
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
		if (status_ != CalibrationStatus::WaitingContinue)
			return;

		if (thread_ptr_)
		{
			if (thread_ptr_->joinable())	// actually not gonna happen
				thread_ptr_->join();
			thread_ptr_.reset();
		}

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
			if (target_index_ != -1 && saved_behavior_ != TrackerBehavior::kInvalid)
			{
				AtomicTracker target = tk_provider_.FindByIndex(target_index_);
				target->set_behavior(saved_behavior_);
			}

			logger_.Info("Calibration process aborted.");
		}

		Reset();
	}

	CalibrationManager::CalibrationStatus CalibrationManager::GetStatus() const
	{
		return status_;
	}

	CalibrationManager::SampleTypes CalibrationManager::GetRequiredSampleType() const
	{
		return sample_type_;
	}

	std::string CalibrationManager::GetStatusAsString() const
	{
		switch (status_)
		{
		case CalibrationStatus::StandBy:
			return CONST_STRING(StandBy);

		case CalibrationStatus::ConfiguringTracker:
			return CONST_STRING(ConfiguringTracker);

		case CalibrationStatus::WaitingContinue:
			return CONST_STRING(WaitingContinue);

		case CalibrationStatus::RecordingIMU:
			return CONST_STRING(RecordingIMU);

		case CalibrationStatus::Calibrating:
			return CONST_STRING(Calibrating);

		default:
			break;
		}

		return "";
	}

	std::string CalibrationManager::GetRequiredSampleTypeAsString() const
	{
		switch (sample_type_)
		{
		case SampleTypes::NegativeZ:
			return CONST_STRING(NegativeZ);

		case SampleTypes::PositiveZ:
			return CONST_STRING(PositiveZ);

		case SampleTypes::NegativeY:
			return CONST_STRING(NegativeY);

		case SampleTypes::PositiveY:
			return CONST_STRING(PositiveY);

		case SampleTypes::NegativeX:
			return CONST_STRING(NegativeX);

		case SampleTypes::PositiveX:
			return CONST_STRING(PositiveX);

		case SampleTypes::Rotational:
			return CONST_STRING(Rotational);

		default:
			break;
		}
		return "";
	}

	void CalibrationManager::Reset()
	{
		accel_calib_.Reset();

		status_ = CalibrationStatus::StandBy;
		sample_type_ = SampleTypes(0);
		target_index_ = -1;
		saved_behavior_ = TrackerBehavior::kInvalid;

		exit_flag_ = false;

		samples_.clear();
		mag_noise_var_ = 0.0f;
		result_ = Calibration{};
	}

	void CalibrationManager::ConfiguringThreadLoop()
	{
		constexpr TrackerBehavior kCalibBehavior
		{
			.active = true,
			.raw = true,
			.led = true
		};

		status_ = CalibrationStatus::ConfiguringTracker;

		// configure target
		{
			AtomicTracker target = tk_provider_.FindByIndex(target_index_);
			saved_behavior_ = target->behavior();
			target->set_behavior(kCalibBehavior.Encode());
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

		status_ = CalibrationStatus::WaitingContinue;
		logger_.Debug("[Calibration] (1/9) Tracker configured.");
	}

	void CalibrationManager::RecordingThreadLoop()
	{
		status_ = CalibrationStatus::RecordingIMU;

		if (sample_type_ == SampleTypes::Rotational)
			required_size_ = kRequiredRotationalSampleSize;
		else
			required_size_ = kRequiredStaticSampleSize;
		
		// record
		samples_.clear();
		while (!exit_flag_)
		{
			{
				// TODO: steady for axis, rotate for rotational checking logic
				AtomicTracker target = tk_provider_.FindByIndex(target_index_);
				samples_.push_back(target->imu_readings());
			}

			if (samples_.size() >= required_size_)
				break;

			std::this_thread::sleep_for(kRecordingInterval);
		}

		// aborted
		if (exit_flag_)
			return;

		HandleSamples();
		if (sample_type_ != SampleTypes::Rotational)
		{
			// record next samples
			sample_type_ = SampleTypes(static_cast<int>(sample_type_) + 1);
			status_ = CalibrationStatus::WaitingContinue;
			logger_.Debug("[Calibration] ({}/9) Sample aquired.", static_cast<int>(sample_type_) + 2);
		}
		else
		{
			// recording finished
			status_ = CalibrationStatus::Calibrating;
			logger_.Debug("[Calibration] (8/9) Calibrating...");
			ApplyCalibration();
		}
	}

	void CalibrationManager::HandleSamples()
	{
		switch (sample_type_)
		{
		case SampleTypes::NegativeZ:
		{
			Vector3 gyro_result = gyro_calib_.CalculateOffset(samples_);
			result_.gyro_offset[0] = gyro_result.x;
			result_.gyro_offset[1] = gyro_result.y;
			result_.gyro_offset[2] = gyro_result.z;

			accel_calib_.AccumulateSample(AccelCalibrator::Axis::ZNegative, samples_);
			mag_noise_var_ = mag_calib_.CalculateNoiseVariance(samples_);
			break;
		}

		case SampleTypes::PositiveZ:
			accel_calib_.AccumulateSample(AccelCalibrator::Axis::ZPositive, samples_);
			break;

		case SampleTypes::NegativeY:
			accel_calib_.AccumulateSample(AccelCalibrator::Axis::YNegative, samples_);
			break;

		case SampleTypes::PositiveY:
			accel_calib_.AccumulateSample(AccelCalibrator::Axis::YPositive, samples_);
			break;

		case SampleTypes::NegativeX:
			accel_calib_.AccumulateSample(AccelCalibrator::Axis::XNegative, samples_);
			break;

		case SampleTypes::PositiveX:
		{
			accel_calib_.AccumulateSample(AccelCalibrator::Axis::XPositive, samples_);
			Matrix accel_result = accel_calib_.CalculateCalibrationMatrix();
			std::copy_n(accel_result.data(), 12, result_.accel_mat);
			break;
		}

		case SampleTypes::Rotational:
		{
			Matrix mag_result = mag_calib_.CalculateCalibrationMatrix(samples_, mag_noise_var_);
			std::copy_n(mag_result.data(), 12, result_.mag_mat);
			break;
		}

		default:
			break;
		}
	}

	void CalibrationManager::ApplyCalibration()
	{
		{
			AtomicTracker target = tk_provider_.FindByIndex(target_index_);
			target->set_behavior(saved_behavior_);
			target->set_calibration(result_);
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

		logger_.Debug("[Calibration] (9/9) Calibration finished.");
		logger_.Info("Calibration finished.");
		Reset();
	}

}	// namespace dkvr