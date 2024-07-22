#include "calibrator/gyro_calibrator.h"

#include <algorithm>
#include <random>
#include <vector>

#include "math/vector.h"
#include "math/matrix.h"

namespace dkvr
{

	Matrix GyroCalibrator::CalculateCalibrationMatrix(const std::vector<Vector3>& gyro_samples, const std::vector<Vector3>& mag_samples, float time_step)
	{
		// reset
		calibration_matrix_.Fill(0);
		for (int i = 0; i < 3; i++)
			calibration_matrix_[i][i] = 1.0f;
		gradient_.Fill(0);

		// create sample set
		int sample_size = std::min(gyro_samples.size(), mag_samples.size());
		std::vector<SampleTuple> sample_set;
		for (int i = 1; i < sample_size; i++)
			sample_set.push_back(SampleTuple{ gyro_samples[i], mag_samples[i - 1], mag_samples[i] });

		// iterate
		std::default_random_engine rng = std::default_random_engine{};
		for (int iter = 0; iter < iteration_; iter++)
		{
			// shuffle sample set
			std::shuffle(sample_set.begin(), sample_set.end(), rng);

			// stochastic gradient descendent
			for (int idx = 1; idx < sample_size; idx += batch_size_)
			{
				// get gradient of batch
				gradient_.Fill(0);
				int count = std::min(batch_size_, sample_size - idx);
				for (int inc = 0; inc < count; inc++)
				{
					SampleTuple& target = sample_set[idx + inc];
					AddGradient(target.gyro, target.old_mag, target.new_mag, time_step);	// not averaged
				}

				// apply gradient
				for (int i = 0; i < 3; i++)
					for (int j = 0; j < 4; j++)
						calibration_matrix_[i][j] -= learn_rate_ * gradient_[i * 4 + j] / count;
			}
		}

		return calibration_matrix_;
	}

	void GyroCalibrator::AddGradient(const Vector3& gyro, const Vector3& old_mag, const Vector3& new_mag, float time_step)
	{
		Vector3 calibrated{ 0 };
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++)
				calibrated[i] += calibration_matrix_[i][j] * gyro[j];
			calibrated[i] += calibration_matrix_[i][3];
			calibrated[i] *= time_step;
		}

		float p1 =  old_mag.x				 - old_mag.z * calibrated.y + old_mag.y * calibrated.z - new_mag.x;
		float p2 =  old_mag.z * calibrated.x + old_mag.y				- old_mag.x * calibrated.z - new_mag.y;
		float p3 = -old_mag.y * calibrated.x + old_mag.x * calibrated.y + old_mag.z - new_mag.z;

		float result[12]{
									   old_mag.z * p2 * gyro.x - old_mag.y * p3 * gyro.x,
									   old_mag.z * p2 * gyro.y - old_mag.y * p3 * gyro.y,
									   old_mag.z * p2 * gyro.z - old_mag.y * p3 * gyro.z,
									   old_mag.z * p2		   - old_mag.y * p3,

			-old_mag.z * p1 * gyro.x + old_mag.x * p3 * gyro.x,
			-old_mag.z * p1 * gyro.y + old_mag.x * p3 * gyro.y,
			-old_mag.z * p1 * gyro.z + old_mag.x * p3 * gyro.z,
			-old_mag.z * p1			 + old_mag.x * p3,

			 old_mag.y * p1 * gyro.x - old_mag.x * p2 * gyro.x,
			 old_mag.y * p1 * gyro.y - old_mag.x * p2 * gyro.y,
			 old_mag.y * p1 * gyro.z - old_mag.x * p2 * gyro.z,
			 old_mag.y * p1			 - old_mag.x * p2
		};

		for (int i = 0; i < 12; i++)
			gradient_[i] += 2 * result[i];
	}

}	// namespace dkvr