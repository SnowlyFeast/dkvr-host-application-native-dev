#include "math/qr_algorithm.h"

#include <cmath>
#include <stdexcept>
#include <vector>

#include "math/matrix.h"
#include "math/vector.h"

namespace dkvr {

	std::vector<EigenPair> QRAlgorithm::GetEigenPairs(Matrix mat)
	{
		if (mat.row() != mat.column())
			throw std::invalid_argument("input matrix is not square.");

		constexpr int iteration_limit = 20;
		constexpr float resolution_limit = 0.001f;

		unsigned long size = mat.row();
		Matrix q, qt;
		Matrix a = mat;
		Matrix ev = Matrix::CreateIdentity(size);

		// iterate QR factorization
		int iteration = 0;
		while (true) {
			q = DecomposeQ(a);
			qt = q.GetTranspose();
			a = qt * a * q;
			ev = ev * q;

			// check precision
			bool done = true;
			for (unsigned long i = 0; i < size; i++) {
				for (unsigned long j = i + 1; j < size; j++) {
					if (abs(a[i][j]) > resolution_limit) {
						done = false;
						break;
					}
				}
				if (!done) break;
			}

			// force end on iteration limit
			if (done || iteration > iteration_limit) break;
			iteration++;
		}

		// eigenvalues  = diag(A)	=> a[i][i]
		// eigenvectors = col(ev)	=> ev.GetColumnVector(i)
		std::vector<EigenPair> vec;
		vec.reserve(size);
		for (unsigned long i = 0; i < size; i++)
			vec.push_back(EigenPair(a[i][i], ev.GetColumnVector(i)));

		return vec;
	}

	Matrix QRAlgorithm::DecomposeQ(const Matrix& mat)
	{
		Matrix q = GetQk(mat);
		if (mat.row() > 2) {
			Matrix q_next = DecomposeQ(mat.GetMinor(0, 0));
			return IdentityFilledMatrixProduct(q, q_next);
		}
		return q;
	}

	Matrix QRAlgorithm::GetQk(const Matrix& mat)
	{
		// column vector x to vector u
		Vector vec = mat.GetColumnVector(0);
		if (vec[0] > 0)
			vec[0] += (float)vec.GetEuclidianNorm();
		else
			vec[0] -= (float)vec.GetEuclidianNorm();

		// gram matrix to vector u
		Matrix gram = Matrix::CreateGramMatrix(vec);

		// calculate Qk
		gram = gram * (-2.0f / vec.GetEuclidianNorm(false));
		for (unsigned long i = 0; i < vec.size(); i++)
			gram[i][i] += 1;

		return gram;
	}

	Matrix QRAlgorithm::IdentityFilledMatrixProduct(const Matrix& lhs, const Matrix& rhs)
	{
		unsigned long size = lhs.row();
		unsigned long gap = lhs.row() - rhs.row();
		Matrix result(size, size);
		result.Fill();
		for (unsigned long i = 0; i < size; i++)
			for (unsigned long j = 0; j < size; j++)
				for (unsigned long k = 0; k < size; k++) {
					if (j < gap)
						result[i][j] += (j == k ? lhs[i][k] : 0);
					else if (k < gap)
						continue;
					else
						result[i][j] += lhs[i][k] * rhs[k - gap][j - gap];
				}
		return result;
	}

}	// namespace dkvr