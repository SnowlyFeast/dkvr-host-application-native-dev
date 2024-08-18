#include "calibrator/common_calibrator.h"

#include "Eigen/Dense"

namespace dkvr
{

	Eigen::Vector3f CommonCalibrator::CalculateNoiseVariance(const std::vector<Eigen::Vector3f>& samples)
	{
		// calculate original variance
		Eigen::Vector3f mean(0, 0, 0);
		for (const Eigen::Vector3f& s : samples)
			mean += s;
		mean /= samples.size();

		Eigen::Array3f var(0, 0, 0);
		for (const Eigen::Vector3f& s : samples)
			var += (s - mean).array().square();
		var /= samples.size();

		return var;
	}

	void CommonCalibrator::TransformSamples(std::vector<Eigen::Vector3f>& samples, const CalibrationMatrix& calib)
	{
		for (Eigen::Vector3f& s : samples)
			s = (calib.transform * s + calib.offset).eval();
	}

	void CommonCalibrator::TransformNoiseVariance(Eigen::Vector3f& noise_var, const CalibrationMatrix& calib)
	{
		Eigen::Array3f diag = calib.transform.diagonal();
		noise_var = (diag.square() * noise_var.array()).eval();
	}

}	// namespace dkvr