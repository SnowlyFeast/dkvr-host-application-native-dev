#include "math/ellipsoid_estimator.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "Eigen/Dense"

namespace dkvr {

	Eigen::Vector3f EllipsoidParameter::GetCenterVector() const
	{
		return	-0.5f * (a.inverse() * b);
	}

	Eigen::Matrix3f EllipsoidParameter::GetTransformationMatrix() const
	{
		Eigen::Vector3f c = GetCenterVector();
		Eigen::Matrix3f a_tild = a * (1.0f / (c.transpose() * a * c - d));

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(a_tild);
		// I don't think this will happen
		if (solver.info() != Eigen::Success)
			return Eigen::Matrix3f{ {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };

		return solver.eigenvectors() * solver.eigenvalues().sqrt() * solver.eigenvectors().transpose();
	}


	// [ Reference ] complete algorithm is available here
	// "Consistent Least Squares Fitting of Ellipsoids." Numerische Mathematik 98 (2004): 177-194.
	EllipsoidParameter EllipsoidEstimator::EstimateEllipsoid(const std::vector<Eigen::Vector3f>& samples, float noise_var)
	{
		constexpr int max_sample_size = 300;

		const int size = std::min(static_cast<int>(samples.size()), max_sample_size);
		Eigen::Matrix<float, 3, max_sample_size> t[5];
		for (int i = 0; i < size; i++)
		{
			t[0].col(i) = Eigen::Vector3f(1, 1, 1);
			t[1].col(i) = samples[i];
			t[2].col(i) = samples[i].array().pow(2) - noise_var;
			t[3].col(i) = samples[i].array().pow(3) - 3 * samples[i].array() * noise_var;
			t[4].col(i) = samples[i].array().pow(4) - 6 * samples[i].array().square() * noise_var + 3 * powf(noise_var, 2);
		}

		constexpr int m[10][2]{ {1, 1}, {1, 2}, {2, 2}, {1, 3}, {2, 3}, {3, 3}, {1, 0}, {2, 0}, {3, 0}, {0, 0} };

		int r[10][10][3]{};
		for (int p = 0; p < 10; p++)
			for (int q = 0; q < 10; q++)
				for (int i = 1; i <= 3; i++)
					r[p][q][i - 1] = (m[p][0] == i) + (m[p][1] == i) + (m[q][0] == i) + (m[q][1] == i);

		float eta[10][10]{};
		for (int p = 0; p < 10; p++)
			for (int q = p; q < 10; q++) {
				float total = 0;
				for (unsigned long k = 0; k < size; k++) {
					float val = 1;
					for (int i = 0; i < 3; i++) {
						int deg = r[p][q][i];
						val *= t[deg](i, k);
					}
					total += val;
				}
				eta[p][q] = total;
			}

		Eigen::Matrix<float, 10, 10> psi;
		for (int p = 0; p < 10; p++)
			for (int q = p; q < 10; q++)
			{
				int multiplier = 1;
				if (p == 1 || p == 3 || p == 4) multiplier *= 2;
				if (q == 1 || q == 3 || q == 4) multiplier *= 2;
				psi(p, q) = eta[p][q] * multiplier;

				if (p == q) continue;
				psi(q, p) = psi(p, q);
			}

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 10, 10>> solver(psi);
		if (solver.info() != Eigen::Success)
			return EllipsoidParameter(Eigen::Matrix3f{ {1, 0, 0}, {0, 1, 0}, {0, 0, 1} }, Eigen::Vector3f(0, 0, 0), 0.0f);

		auto max = std::max_element(solver.eigenvalues().begin(), solver.eigenvalues().end());
		int index = std::distance(solver.eigenvalues().begin(), max);

		Eigen::Vector<float, 10> b_als = solver.eigenvectors().col(index);
		Eigen::Matrix3f a{ 
			{b_als[0], b_als[1], b_als[3]},
			{b_als[1], b_als[2], b_als[4]},
			{b_als[3], b_als[4], b_als[5]}
		};
		Eigen::Vector3f b{ b_als[6], b_als[7], b_als[8] };
		float d = b_als[9];
		
		return EllipsoidParameter(a, b, d);
	}

}	// namespace dkvr