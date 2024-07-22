#include "calibrator/mag_calibrator.h"

#include <cmath>
#include <vector>

#include "math/matrix.h"
#include "math/vector.h"
#include "math/ellipsoid_estimator.h"

namespace dkvr
{
	float MagCalibrator::CalculateNoiseVariance(const std::vector<Vector3>& samples)
	{
		Vector3 mean{ 0 };
		for (const Vector3& s : samples)
			mean += s;
		mean /= samples.size();

		Vector3 var{ 0 };
		for (const Vector3& s : samples)
		{
			Vector3 diff = s - mean;
			var.x += diff.x * diff.x;
			var.y += diff.y * diff.y;
			var.z += diff.z * diff.z;
		}
		var /= samples.size();

		return (var.x + var.y + var.z) / 3;
	}

	Matrix MagCalibrator::CalculateCalibrationMatrix(const std::vector<Vector3>& samples, float noise_variance)
	{
		std::vector<Vector3> raw;
		raw.reserve(samples.size());
		for (const Vector3& v : samples)
			raw.push_back(v);

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