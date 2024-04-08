#pragma once

#include <vector>

#include "math/matrix.h"
#include "math/vector.h"

namespace dkvr {

	struct EigenPair
	{
		float eigenvalue;
		Vector eigenvector;

		EigenPair(float val, const Vector& vec) : eigenvalue(val), eigenvector(vec) { }
	};

	class QRAlgorithm
	{
	public:
		static std::vector<EigenPair> GetEigenPairs(Matrix mat);

	private:
		static Matrix DecomposeQ(const Matrix& mat);
		static Matrix GetQk(const Matrix& mat);
		static Matrix IdentityFilledMatrixProduct(const Matrix& lhs, const Matrix& rhs);
	};

}	// namespace dkvr