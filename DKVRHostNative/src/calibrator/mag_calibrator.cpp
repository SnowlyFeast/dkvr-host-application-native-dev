#include "calibrator/mag_calibrator.h"

#include <vector>

#include "Eigen/Dense"

#include "calibrator/common_calibrator.h"
#include "math/ellipsoid_estimator.h"
#include "tracker/tracker_data.h"

namespace dkvr
{

	namespace
	{
		bool IsRotationalConstraintSatisfied(const RawDataSet& current, const Eigen::Vector3f& prev)
		{
			constexpr float minimum_angle = 3.0f;

			Eigen::Vector3f v_cur{ current.mag[0], current.mag[1], current.mag[2] };
			float cos = v_cur.normalized().dot(prev.normalized());
			return cos > cosf(minimum_angle);
		}
	}

	void MagCalibrator::Reset()
	{
		mag_samples.clear();
	}

	void MagCalibrator::Accumulate(SampleType type, const std::vector<RawDataSet>& samples)
	{
		constexpr size_t limit = 300;

		// calculate noise variance once
		if (type == SampleType::XPositive)
		{
			size_t size = std::min(limit, samples.size());

			std::vector<Eigen::Vector3f> vec;
			vec.reserve(size);
			for (int i = 0; i < size; i++)
				vec.emplace_back(samples[i].mag[0], samples[i].mag[1], samples[i].mag[2]);

			noise_var_ = CommonCalibrator::CalculateNoiseVariance(vec);
		}
		// only interested with rotational samples
		else if (type == SampleType::Rotational)
		{
			// prepare well-spaced samples
			mag_samples.emplace_back(samples[0].mag[0], samples[0].mag[1], samples[0].mag[2]);
			for (int i = 1; i < samples.size(); i++)
			{
				if (IsRotationalConstraintSatisfied(samples[i], mag_samples.back()))
					mag_samples.emplace_back(samples[i].mag[0], samples[i].mag[1], samples[i].mag[2]);

				if (mag_samples.size() >= limit)
					break;
			}
		}

	}

	void MagCalibrator::Calculate()
	{
		// calculate calibration matrix
		EllipsoidParameter param = EllipsoidEstimator::EstimateEllipsoid(mag_samples, noise_var_.norm());
		Eigen::Matrix3f transform = param.GetTransformationMatrix();
		Eigen::Vector3f offset = -transform * param.GetCenterVector();

		result_.transform = transform;
		result_.offset = offset;
		CommonCalibrator::TransformNoiseVariance(noise_var_, result_);
	}

}	// namespace dkvr