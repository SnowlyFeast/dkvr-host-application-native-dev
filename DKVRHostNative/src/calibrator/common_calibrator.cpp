#include "calibrator/common_calibrator.h"

namespace dkvr
{
	void CommonCalibrator::CalibrateSamples(std::vector<Vector3>& samples, const Matrix& calibration_matrix)
	{
		for (Vector3& s : samples)
		{
			float calibrated[3]{ 0 };
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
					calibrated[i] += s[j] * calibration_matrix[i][j];
				calibrated[i] += calibration_matrix[i][3];
			}

			for (int i = 0; i < 3; i++)
				s[i] = calibrated[i];
		}
	}

	Vector3 CommonCalibrator::CalculateNoiseVariance(const std::vector<Vector3>& samples, const Matrix& calibration_matrix)
	{
		// calculate original variance
		Vector3 mean{ 0 };
		for (const Vector3& s : samples)
			mean += s;
		mean /= samples.size();

		Vector3 var{ 0 };
		for (const Vector3& s : samples)
		{
			Vector3 diff = s - mean;
		}
		var /= samples.size();

		// transform variance
		Vector3 result{ 0 };
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				result[i] += var[j] * calibration_matrix[i][j] * calibration_matrix[i][j];

		return result;
	}

}	// namespace dkvr