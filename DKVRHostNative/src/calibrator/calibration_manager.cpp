#include "calibrator/calibration_manager.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include "tracker/tracker.h"
#include "tracker/tracker_configuration.h"
#include "calibrator/common_calibrator.h"

#define GLUE(x, y)			(x ## y)
#define CONST_STRING(name)	GLUE(kString, name)

namespace dkvr
{
	constexpr size_t kRequiredGyroSampleSize = 1000;
	constexpr size_t kRequiredAccelSampleSize = 100;
	constexpr size_t kRequiredMagSampleSize = 300;
	constexpr float kStaticSampleLinearAccelerationLimit = 0.2f;
	constexpr float kRotationalSampleMinimumAngle = 3;
	constexpr std::chrono::milliseconds kValidationInterval(500);

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
		sample_type_(SampleTypes::NegativeZ), target_index_(-1), saved_behavior_(TrackerBehavior::kInvalid), saved_calibration_{}, 
		thread_ptr_(nullptr), exit_flag_(false), gyro_samples_(), accel_samples_(), mag_samples1_(), mag_samples2_(),
		mag_noise_var_(0.0f), result_matrix_{}, result_noise_var_{}, tk_provider_(tk_provider)
	{
		gyro_samples_.reserve(kRequiredGyroSampleSize);
		accel_samples_.reserve(kRequiredAccelSampleSize);
		mag_samples1_.reserve(kRequiredMagSampleSize);
		mag_samples2_.reserve(kRequiredGyroSampleSize);
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
				target->set_calibration(saved_calibration_);
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

		gyro_samples_.clear();
		accel_samples_.clear();
		mag_samples1_.clear();
		mag_samples2_.clear();
		mag_noise_var_ = 0.0f;
		result_matrix_ = CalibrationMatrix{};
		result_noise_var_ = NoiseVariance{};
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

			CalibrationMatrix default_calib{};
			default_calib.Reset();
			saved_calibration_ = target->calibration();
			target->set_calibration(default_calib);
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

		// begin sample record
		accel_samples_.clear();
		while (!exit_flag_)
		{
			// get IMU readings if updated
			bool yield = false;
			IMUReadings readings;
			{
				AtomicTracker target = tk_provider_.FindByIndex(target_index_);
				if (target->IsImuReadingsUpdated())
					readings = target->imu_readings();
				else
					yield = true;
			}
			if (yield)
			{
				std::this_thread::yield();
				continue;
			}

			// handle by sample type
			if (sample_type_ != SampleTypes::Rotational)
			{	
				// first sample
				if (accel_samples_.size() == 0)
				{
					accel_samples_.push_back(readings.acc);
					continue;
				}

				// check static sample constraint
				Vector3 diff = readings.acc - accel_samples_.back();
				if (diff.ToVector().GetEuclidianNorm() > kStaticSampleLinearAccelerationLimit)
					continue;
				accel_samples_.push_back(readings.acc);
				
				if (sample_type_ == SampleTypes::PositiveX)	// for magnetic variance calculation
					mag_samples1_.push_back(readings.mag);

				if (accel_samples_.size() >= kRequiredAccelSampleSize)
					break;
			}
			else
			{	// rotational samples

				// continuously accumulate for gyro calibration
				if (gyro_samples_.size() < kRequiredGyroSampleSize)
				{
					gyro_samples_.push_back(readings.gyr);
					mag_samples2_.push_back(readings.mag);
				}

				// first sample
				if (mag_samples1_.size() == 0)
				{
					mag_samples1_.push_back(readings.mag);
					continue;
				}

				// check mag minimum rotational constraint
				float cos = readings.mag.ToVector().GetNormalized() * mag_samples1_.back().ToVector().GetNormalized();
				if (cos > cosf(kRotationalSampleMinimumAngle))
					continue;
				mag_samples1_.push_back(readings.mag);

				if (gyro_samples_.size() >= kRequiredGyroSampleSize && mag_samples1_.size() >= kRequiredMagSampleSize)
					break;
			}
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
			logger_.Debug("[Calibration] ({}/9) Sample aquired.", static_cast<int>(sample_type_) + 1);
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
			accel_calib_.AccumulateSample(AccelCalibrator::Axis::ZNegative, accel_samples_);
			break;

		case SampleTypes::PositiveZ:
			accel_calib_.AccumulateSample(AccelCalibrator::Axis::ZPositive, accel_samples_);
			break;

		case SampleTypes::NegativeY:
			accel_calib_.AccumulateSample(AccelCalibrator::Axis::YNegative, accel_samples_);
			break;

		case SampleTypes::PositiveY:
			accel_calib_.AccumulateSample(AccelCalibrator::Axis::YPositive, accel_samples_);
			break;

		case SampleTypes::NegativeX:
			accel_calib_.AccumulateSample(AccelCalibrator::Axis::XNegative, accel_samples_);
			break;

		case SampleTypes::PositiveX:
		{
			// calculate gyro calibration matrix and noise variance
			accel_calib_.AccumulateSample(AccelCalibrator::Axis::XPositive, accel_samples_);
			Matrix matrix = accel_calib_.CalculateCalibrationMatrix();
			std::copy_n(matrix.data(), 12, result_matrix_.acc);

			Vector3 noise_var = CommonCalibrator::CalculateNoiseVariance(accel_samples_, matrix);
			std::copy_n(&noise_var.x, 3, result_noise_var_.acc);

			// caluclate 1-float mag variance
			mag_noise_var_ = mag_calib_.CalculateNoiseVariance(mag_samples1_);
			mag_samples1_.clear();
			break;
		}

		case SampleTypes::Rotational:
		{
			// calculate mag calibration matrix and noise variance
			Matrix mag_matrix = mag_calib_.CalculateCalibrationMatrix(mag_samples1_, mag_noise_var_);
			std::copy_n(mag_matrix.data(), 12, result_matrix_.mag);

			Vector3 mag_noise_var = CommonCalibrator::CalculateNoiseVariance(mag_samples1_, mag_matrix);
			std::copy_n(&mag_noise_var.x, 3, result_noise_var_.mag);

			// calculate gyro calibration matrix
			CommonCalibrator::CalibrateSamples(mag_samples2_, mag_matrix);
			Matrix gyro_matrix = gyro_calib_.CalculateCalibrationMatrix(gyro_samples_, mag_samples2_, 0.01f);	// TODO: pull 0.01f to constant
			std::copy_n(gyro_matrix.data(), 12, result_matrix_.gyr);
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
			target->set_calibration(result_matrix_);
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