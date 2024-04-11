#include "calibrator/mag_calibrator.h"

#include <cmath>
#include <vector>

#include "math/matrix.h"
#include "math/vector.h"
#include "math/ellipsoid_estimator.h"

namespace dkvr
{
	float MagCalibrator::CalculateNoiseVariance(const std::vector<IMUReadings>& samples)
	{
		float mean = 0;
		for (const IMUReadings& s : samples)
		{
			const Vector3& v = s.mag;
			mean += sqrtf((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
		}
		mean /= samples.size();

		float var = 0;
		for (const IMUReadings& s : samples)
		{
			const Vector3& v = s.mag;
			var += powf(sqrtf((v.x * v.x) + (v.y * v.y) + (v.z * v.z)) - mean, 2);
		}
		var /= samples.size();

		return var;
	}

	Matrix MagCalibrator::CalculateCalibrationMatrix(const std::vector<IMUReadings>& samples, float noise_variance)
	{
		std::vector<Vector3> raw;
		raw.reserve(samples.size());
		for (const IMUReadings& v : samples)
			raw.push_back(v.mag);

		EllipsoidParameter param = EllipsoidEstimator::EstimateEllipsoid(raw, noise_variance);

		Matrix result(3, 4);
		Matrix tf = param.GetTransformationMatrix();
		Vector offset = tf * param.GetCenterVector();

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
				result[i][j] = tf[i][j];
			result[i][3] = -offset[i];
		}

		return result;
	}

}	// namespace dkvr