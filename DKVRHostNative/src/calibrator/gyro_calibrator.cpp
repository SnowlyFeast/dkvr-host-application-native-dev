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
		result_.transform.setIdentity();
		result_.offset.setZero();
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
			Reset();
			RunGradientDescent(samples);

			// transform noise variance
			CommonCalibrator::TransformNoiseVariance(noise_var_, result_);
		}
	}

	void GyroCalibrator::RunGradientDescent(const std::vector<RawDataSet>& samples)
	{
		int sample_size = samples.size() - 1;

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
			std::shuffle(sample_set.begin(), sample_set.end(), rng);

			// stochastic gradient descendent
			for (int idx = 0; idx < sample_size; idx += batch_size_)
			{
				// get gradient of batch
				gradient_.setZero();
				int count = std::min(batch_size_, sample_size - idx);
				for (int inc = 0; inc < count; inc++)
					AddGradient(sample_set[idx + inc]);	// not averaged
				gradient_ *= (learn_rate_ / count);

				// apply gradient
				Eigen::Matrix<float, 3, 4> result = gradient_.reshaped<Eigen::AutoOrder>(3, 4);
				result_.transform += result.topLeftCorner<3, 3>();
				result_.offset += result.col(3);
			}
		}
	}

	void GyroCalibrator::AddGradient(const SampleTuple& tuple)
	{
		Eigen::Vector3f cal = (result_.transform * tuple.gyro + result_.offset) * time_step_;
		Eigen::Matrix3f rot{
			{     1.0f,  cal.z(), -cal.y() },
			{ -cal.z(),     1.0f,  cal.x() },
			{  cal.y(), -cal.x(),     1.0f }
		};
		
		Eigen::Vector3f djdp = (rot * tuple.old_mag - tuple.new_mag) * 2;
		Eigen::Matrix3f dpdy{
			{                  0, -tuple.old_mag.z(),  tuple.old_mag.y() },
			{  tuple.old_mag.z(),                  0, -tuple.old_mag.x() },
			{ -tuple.old_mag.y(),  tuple.old_mag.x(),                  0 }
		};
		Eigen::Matrix<float, 3, 12> dydw{
			{ cal.x(), cal.y(), cal.z(), 1.0f,            0, 0, 0, 0,                      0, 0, 0, 0           },
			{            0, 0, 0, 0,           cal.x(), cal.y(), cal.z(), 1.0f,            0, 0, 0, 0           },
			{            0, 0, 0, 0,                      0, 0, 0, 0,           cal.x(), cal.y(), cal.z(), 1.0f },
		};

		gradient_ += djdp.transpose() * dpdy * dydw;
	}

}	// namespace dkvr