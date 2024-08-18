#include "calibrator/accel_calibrator.h"

#include <algorithm>

#include "Eigen/Dense"

#include "calibrator/common_calibrator.h"

namespace dkvr
{

	namespace 
	{
		enum class Axis
		{
			XPositive,
			XNegative,
			YPositive,
			YNegative,
			ZPositive,
			ZNegative,
			Invalid
		};

		Axis ConvertToInternalAxis(SampleType type)
		{
			switch (type)
			{
			case SampleType::ZNegative:
				return Axis::ZNegative;

			case SampleType::ZPositive:
				return Axis::ZPositive;

			case SampleType::YNegative:
				return Axis::YNegative;

			case SampleType::YPositive:
				return Axis::YPositive;

			case SampleType::XNegative:
				return Axis::XNegative;

			case SampleType::XPositive:
				return Axis::XPositive;

			default:
			case SampleType::Rotational:
				return Axis::Invalid;
			}
		}
	}
	
	
	void AccelCalibrator::Reset()
	{
		std::fill(accumulated_, accumulated_ + 6, 0);
		std::fill(samples_avg_, samples_avg_ + 6, Eigen::Vector3f(0, 0, 0));
	}

	void AccelCalibrator::Accumulate(SampleType type, const std::vector<RawDataSet>& samples)
	{
		Axis axis = ConvertToInternalAxis(type);
		if (axis == Axis::Invalid)
			return;

		int idx = static_cast<int>(axis);
		if (accumulated_[idx])
			return;

		// averaged
		Eigen::Vector3f mean(0, 0, 0);
		for (const RawDataSet& s : samples)
		{
			mean.x() += s.acc[0];
			mean.y() += s.acc[1];
			mean.z() += s.acc[2];
		}

		samples_avg_[idx] = mean / samples.size();
		accumulated_[idx] = true;

		// calculate noise variance once
		if (axis == Axis::XPositive)
		{
			std::vector<Eigen::Vector3f> vec;
			vec.reserve(samples.size());
			for (const RawDataSet& s : samples)
				vec.emplace_back(s.acc[0], s.acc[1], s.acc[2]);

			noise_var_ = CommonCalibrator::CalculateNoiseVariance(vec);
		}
	}

	void AccelCalibrator::Calculate()
	{
		// all axis are ready
		for (bool b : accumulated_)
			if (!b) return;

		// calculate transform matrix
		Eigen::Matrix<float, 6, 4> raw;
		for (int i = 0; i < 6; i++)
		{
			raw.block<1, 3>(i, 0) = samples_avg_[i];
			raw(i, 3) = 1.0f;
		}

		Eigen::Matrix<float, 6, 3> expected{
			{ 1.0f,     0,     0},
			{-1.0f,     0,     0},
			{ 0,     1.0f,     0},
			{ 0,    -1.0f,     0},
			{ 0,        0,  1.0f},
			{ 0,        0, -1.0f}
		};

		// solve linear least square (using normal equations)
		Eigen::Matrix<float, 3, 4> solution = (raw.transpose() * raw).ldlt().solve(raw.transpose() * expected).transpose();

		result_.transform = solution.topLeftCorner<3, 3>();
		result_.offset = solution.col(3);
		CommonCalibrator::TransformNoiseVariance(noise_var_, result_);
	}

}	// namespace dkvr