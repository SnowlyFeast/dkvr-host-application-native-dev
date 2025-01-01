#include "calibrator/gyro_calibrator.h"

#include <algorithm>
#include <random>
#include <vector>

#include "Eigen/Dense"

#include "calibrator/common_calibrator.h"
#include "tracker/tracker_data.h"

namespace dkvr
{

	namespace
	{
        //constexpr float kDegToRad = 2.0 * EIGEN_PI / 360.0;
        //constexpr float kRadToDeg = 1.0 / kDegToRad;

		Eigen::Vector3f TransformSample(const CalibrationMatrix& calib, const Vector3f& sample)
		{
			Eigen::Vector3f s(sample[0], sample[1], sample[2]);
			return calib.transform * s + calib.offset;
		}

		Eigen::Matrix3f SkewSymmetrize(Eigen::Vector3f vec)
		{
			return Eigen::Matrix3f{
				{      0  , -vec.z(),  vec.y() },
				{  vec.z(),      0  , -vec.x() },
				{ -vec.y(),  vec.x(),      0   }
			};
		}

		Eigen::Vector3f ThetaFunction(Eigen::Vector3f calibrated_gyr, Eigen::Vector3f old_mag, float time_step)
		{
			return (Eigen::Matrix3f::Identity() - time_step * SkewSymmetrize(calibrated_gyr))* old_mag;
		}
	}
	
	void GyroCalibrator::Reset()
	{
		uncalibrated_set_.clear();
		result_.transform.setIdentity();
		result_.offset.setZero();
		progress_perc_ = 0;
	}

	void GyroCalibrator::Accumulate(SampleType type, const std::vector<RawDataSet>& samples)
	{
		constexpr size_t limit = 300;

		// calculate noise variance once
		if (type == SampleType::XPositive)
		{
			size_t size = std::min(limit, samples.size());

			std::vector<Eigen::Vector3f> vec;
			vec.reserve(size);
			for (int i = 0; i < size; i++)
				vec.emplace_back(samples[i].gyr[0], samples[i].gyr[1], samples[i].gyr[2]);

			noise_var_ = CommonCalibrator::CalculateNoiseVariance(vec);

			// set averaged as initial offset
			Eigen::Vector3f avg{ 0, 0, 0 };
			for (const Eigen::Vector3f& s : vec)
				avg += s;
			avg /= vec.size();

			result_.offset = -avg;
		}
		// only interested with rotational samples
		else if (type == SampleType::Rotational)
		{
			uncalibrated_set_ = samples;
		}
	}

	void GyroCalibrator::Calculate()
	{
		calculating_ = true;
		int sample_size = uncalibrated_set_.size();

		// create sample set
		samples_.clear();
		samples_.reserve(sample_size);

		Eigen::Vector3f gyr_k1{ uncalibrated_set_[1].gyr[0], uncalibrated_set_[1].gyr[1], uncalibrated_set_[1].gyr[2] };
		Eigen::Vector3f mag_k0 = TransformSample(mag_calib_, uncalibrated_set_[0].mag);
		Eigen::Vector3f mag_k1 = TransformSample(mag_calib_, uncalibrated_set_[1].mag);
		samples_.emplace_back(gyr_k1, mag_k0, mag_k1);

		for (int i = 2; i < sample_size; i++)
		{
			Eigen::Vector3f gyr{ uncalibrated_set_[i].gyr[0], uncalibrated_set_[i].gyr[1], uncalibrated_set_[i].gyr[2] };
			Eigen::Vector3f& old_mag = samples_.back().new_mag;
			Eigen::Vector3f new_mag = TransformSample(mag_calib_, uncalibrated_set_[i].mag);
			samples_.emplace_back(gyr, old_mag, new_mag);
		}

		RunGradientDescent();

		// post process
		CommonCalibrator::TransformNoiseVariance(noise_var_, result_);
		calculating_ = false;
	}

	void GyroCalibrator::RunGradientDescent()
	{
		int sample_size = samples_.size();

		// iterate
		std::default_random_engine rng = std::default_random_engine{};
		for (int iter = 0; iter < iteration_; iter++)
		{
			// shuffle sample set
			std::shuffle(samples_.begin(), samples_.end(), rng);

			// stochastic gradient descendent
			for (int idx = 0; idx < sample_size; idx += batch_size_)
			{
				// get gradient of batch
				gradient_.setZero();
				int count = std::min(batch_size_, sample_size - idx);
				for (int inc = 0; inc < count; inc++)
					AddGradient(samples_[idx + inc]);	// not averaged
				gradient_ *= (learn_rate_ / count);

				// apply gradient
				Eigen::Matrix<float, 3, 4> result = gradient_.reshaped(3, 4);
				result_.transform -= result.topLeftCorner<3, 3>();
				result_.offset -= result.col(3);
			}

			// set progress
			progress_perc_ = static_cast<int>(iter * 100.0 / iteration_);
		}
	}

	void GyroCalibrator::AddGradient(const SampleTuple& tuple)
	{
		Eigen::Vector3f calibrated_gyr = result_.transform * tuple.gyro + result_.offset;

		Eigen::Vector3f             dj_df = 2 * ThetaFunction(calibrated_gyr, tuple.old_mag, time_step_) - tuple.new_mag;
		Eigen::Matrix3f             df_dw = time_step_ * SkewSymmetrize(tuple.old_mag);
		Eigen::Matrix<float, 3, 12> dw_dt;
		dw_dt.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * tuple.gyro.x();
		dw_dt.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity() * tuple.gyro.y();
		dw_dt.block<3, 3>(0, 6) = Eigen::Matrix3f::Identity() * tuple.gyro.z();
		dw_dt.block<3, 3>(0, 9) = Eigen::Matrix3f::Identity();

		gradient_ += dj_df.transpose() * df_dw * dw_dt;
	}

}	// namespace dkvr