#include "math/ellipsoid_estimator.h"

#include <cmath>
#include <vector>

#include "math/matrix.h"
#include "math/qr_algorithm.h"
#include "math/vector.h"

namespace dkvr {

	Vector EllipsoidParameter::GetCenterVector() const
	{
		return  -0.5f * (a.GetInverse() * b);
	}

	Matrix EllipsoidParameter::GetTransformationMatrix() const
	{
		Vector c = GetCenterVector();
		Matrix a_tild = a * (1.0f / (c * (a * c) - d));

		std::vector<EigenPair> eigen_pairs(QRAlgorithm::GetEigenPairs(a_tild));

		Matrix sigma(3, 3);
		sigma.Fill();
		for (unsigned long i = 0; i < 3; i++)
			sigma[i][i] = sqrt(eigen_pairs[i].eigenvalue);

		Matrix u(3, 3);
		for (unsigned long i = 0; i < 3; i++) {
			Vector v = eigen_pairs[i].eigenvector.GetNormalized();
			for (unsigned long j = 0; j < 3; j++)
				u[j][i] = v[j];
		}

		return u * sigma * u.GetTranspose();
	}

	EllipsoidParameter EllipsoidEstimator::EstimateEllipsoid(const std::vector<Vector3>& samples, float noise_var)
	{
		const unsigned long size = std::min(static_cast<unsigned long>(samples.size()), 300UL);

		std::vector<Matrix> t;
		t.reserve(5);
		for (int i = 0; i < 5; i++)
			t.emplace_back(3, 300);

		for (unsigned long i = 0; i < 3; i++)
			for (unsigned long j = 0; j < size; j++) {
				t[0][i][j] = 1;
				t[1][i][j] = samples[j][i];
				t[2][i][j] = powf(samples[j][i], 2) - noise_var;
				t[3][i][j] = powf(samples[j][i], 3) - 3 * samples[j][i] * noise_var;
				t[4][i][j] = powf(samples[j][i], 4) - 6 * powf(samples[j][i], 2) * noise_var + 3 * powf(noise_var, 2);
			}

		constexpr int m[10][2]{ {1, 1},{1, 2},{2, 2},{1, 3},{2, 3},{3, 3},{1, 0},{2, 0},{3, 0},{0, 0} };

		int r[10][10][3]{};
		for (int p = 0; p < 10; p++)
			for (int q = 0; q < 10; q++)
				for (int i = 1; i <= 3; i++)
					r[p][q][i - 1] = (m[p][0] == i) + (m[p][1] == i) + (m[q][0] == i) + (m[q][1] == i);

		float eta[10][10]{};
		for (int p = 0; p < 10; p++)
			for (int q = p; q < 10; q++) {
				float tot = 0;
				for (unsigned long k = 0; k < size; k++) {
					float val = 1;
					for (int i = 0; i < 3; i++) {
						int deg = r[p][q][i];
						val *= t[deg][i][k];
					}
					tot += val;
				}
				eta[p][q] = tot;
			}

		Matrix psi(10, 10);
		for (unsigned long p = 0; p < 10; p++)
			for (unsigned long q = p; q < 10; q++) {
				bool p_off_diag = (p == 1 || p == 3 || p == 4);
				bool q_off_diag = (q == 1 || q == 3 || q == 4);

				if (p_off_diag && q_off_diag)
					psi[p][q] = 4 * eta[p][q];
				else if (!p_off_diag && !q_off_diag)
					psi[p][q] = eta[p][q];
				else
					psi[p][q] = 2 * eta[p][q];

				if (p == q) continue;
				psi[q][p] = psi[p][q];
			}
		std::vector<EigenPair> eigens(QRAlgorithm::GetEigenPairs(psi));
		int index = 0;
		for (int i = 1; i < 10; i++) {
			if (fabs(eigens[i].eigenvalue) < fabs(eigens[index].eigenvalue))
				index = i;
		}

		Vector b_als = eigens[index].eigenvector.GetNormalized();
		Matrix a(3, 3);
		Vector b(3);
		float d;

		a[0][0] = b_als[0];
		a[0][1] = b_als[1];
		a[0][2] = b_als[3];
		a[1][0] = a[0][1];
		a[1][1] = b_als[2];
		a[1][2] = b_als[4];
		a[2][0] = a[0][2];
		a[2][1] = a[1][2];
		a[2][2] = b_als[5];

		b[0] = b_als[6];
		b[1] = b_als[7];
		b[2] = b_als[8];

		d = b_als[9];

		return EllipsoidParameter(a, b, d);
	}

}	// namespace dkvr