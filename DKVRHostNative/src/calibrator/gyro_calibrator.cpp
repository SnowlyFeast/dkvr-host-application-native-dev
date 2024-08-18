#include "calibrator/gyro_calibrator.h"

#include <algorithm>
#include <random>
#include <vector>

#include "Eigen/Dense"

#include "calibrator/common_calibrator.h"
#include "tracker/tracker_data.h"

namespace dkvr
{
	
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
		std::vector<SampleTuple> sample_set;
		sample_set.reserve(sample_size);
		for (int i = 0; i < sample_size; i++)
			sample_set.emplace_back(samples[i + 1].gyr, samples[i].mag, samples[i + 1].mag);

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