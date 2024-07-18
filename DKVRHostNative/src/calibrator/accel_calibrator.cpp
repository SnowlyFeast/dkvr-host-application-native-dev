#include "calibrator/accel_calibrator.h"

#include <algorithm>

#include "math/matrix.h"
#include "math/vector.h"
#include "tracker/tracker_imu.h"

namespace dkvr
{
	
	void AccelCalibrator::Reset()
	{
		std::fill(accumulated_, accumulated_ + 6, 0);
		std::fill(samples_avg_, samples_avg_ + 6, Vector3());
	}

	void AccelCalibrator::AccumulateSample(Axis axis, const std::vector<IMUReadings>& samples)
	{
		int idx = static_cast<int>(axis);
		if (accumulated_[idx])
			return;

		Vector3& mean = samples_avg_[idx];
		for (const IMUReadings& s : samples)
			mean += s.acc;
		mean /= samples.size();

		accumulated_[idx] = true;
	}

	Matrix AccelCalibrator::CalculateCalibrationMatrix()
	{
		// samples are required from every axis
		for (bool b : accumulated_)
			if (!b)	return Matrix();

		Matrix raw(6, 4);
		for (int i = 0; i < 6; i++)
		{
			raw[i][0] = samples_avg_[i].x;
			raw[i][1] = samples_avg_[i].y;
			raw[i][2] = samples_avg_[i].z;
			raw[i][3] = 1.0f;
		}

		Matrix expected(6, 3);
		expected.Fill(0);
		expected[0][0] =  1.0f;	// X+ upward
		expected[1][0] = -1.0f;	// X- upward
		expected[2][1] =  1.0f;	// Y+ upward
		expected[3][1] = -1.0f;	// Y- upward
		expected[4][2] =  1.0f;	// Z+ upward
		expected[5][2] = -1.0f;	// Z- upward
		
		// least squre method
		Matrix raw_trans = raw.GetTranspose();
		Matrix result = (raw_trans * raw).GetInverse() * raw_trans * expected;

		return result.GetTranspose();	// to row major
	}

	Vector3 AccelCalibrator::CalculateNoiseVariance(const std::vector<IMUReadings>& samples, const Matrix& calibration_matrix)
	{
		Vector3 mean{ 0 };
		for (const IMUReadings& s : samples)
			mean += s.acc;
		mean /= samples.size();

		Vector3 var{ 0 };
		for (const IMUReadings& s : samples)
		{
			Vector3 diff = s.acc - mean;
			mean.x += diff.x * diff.x;
			mean.y += diff.y * diff.y;
			mean.z += diff.z * diff.z;
		}
		var /= samples.size();
		
		// transform variance
		Vector3 result{ 0 };
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				result[i] += var[j] * powf(calibration_matrix[i][j], 2);

		return result;
	}

}	// namespace dkvr