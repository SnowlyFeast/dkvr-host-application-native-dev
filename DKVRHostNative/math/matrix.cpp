#include "matrix.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "vector.h"

namespace dkvr {

	Matrix Matrix::CreateIdentity(unsigned long size)
	{
		Matrix mat(size, size);
		mat.Fill();
		for (unsigned long i = 0; i < size; i++)
			mat[i][i] = 1;
		return mat;
	}

	Matrix Matrix::CreateGramMatrix(Vector vec)
	{
#	ifndef DKVR_MATH_MAT_SIZE_LIMIT_NOTHROW
		constexpr unsigned long lim = std::numeric_limits<unsigned long>::max();
		if (vec.size() > lim)
			throw std::overflow_error("vector size exceeds limit of unsigned long.");
#	endif
		unsigned long size = static_cast<unsigned long>(vec.size());
		Matrix mat(size, size);
		for (unsigned long i = 0; i < size; i++) {
			mat[i][i] = vec[i] * vec[i];
			for (unsigned long j = i + 1; j < size; j++) {
				mat[i][j] = vec[i] * vec[j];
				mat[j][i] = mat[i][j];
			}
		}
		return mat;
	}

	Matrix::Matrix(const Vector& vec) : row_(vec.size()), column_(0), data_(row_, 0.0f)
	{
		std::copy_n(vec.data(), row_, data_.data());
	}

	void Matrix::Fill(float values)
	{
		std::fill_n(data_.data(), data_.size(), values);
	}

	Vector Matrix::GetColumnVector(unsigned long c) const
	{
		Vector vec(row_);
		for (unsigned long i = 0; i < row_; i++)
			vec[i] = (*this)[i][c];
		return vec;
	}

	Matrix Matrix::GetTranspose() const
	{
		Matrix mat(column_, row_);
		for (unsigned long i = 0; i < row_; i++)
			for (unsigned long j = 0; j < column_; j++)
				mat[j][i] = (*this)[i][j];
		return mat;
	}

	Matrix Matrix::GetMinor(unsigned long r, unsigned long c) const
	{
		Matrix mat(row_ - 1, column_ - 1);
		unsigned long row_index = 0;
		for (unsigned long i = 0; i < row_; i++) {
			if (i == r) continue;
			std::copy_n((*this)[i], c, mat[row_index]);
			std::copy_n(&(*this)[i][c + 1], column_ - c - 1, &mat[row_index][c]);
			row_index++;
		}
		return mat;
	}

	float Matrix::GetDeterminant() const
	{
#ifndef DKVR_MATH_MAT_ARITHMETIC_NOTHROW
		if (row_ != column_)
			throw std::domain_error("matrix is not square.");
#endif

		// 2x2
		if (row_ == 2)
			return data_[0] * data_[3] - data_[1] * data_[2];

		// otherwise
		float det = 0;
		for (unsigned long c = 0; c < column_; c++)
			det += data_[c] * GetMinor(0, c).GetDeterminant() * (c % 2 == 0 ? 1 : -1);

		return det;
	}

	Matrix Matrix::GetInverse() const
	{
		float det = GetDeterminant();
#ifndef DKVR_MATH_MAT_ARITHMETIC_NOTHROW
		if (det == 0)
			throw std::domain_error("cannot inverse singular matrix.");
#endif
		Matrix cof(row_, column_);
		for (unsigned long i = 0; i < row_; i++)
			for (unsigned long j = 0; j < column_; j++)
				cof[i][j] = GetMinor(i, j).GetDeterminant() * ((i + j) % 2 == 0 ? 1 : -1) / det;

		return cof.GetTranspose();
	}


	Matrix& Matrix::operator=(const Matrix& mat) noexcept
	{
		row_ = mat.row_;
		column_ = mat.column_;
		data_ = std::vector<float>(mat.data_);
		return *this;
	}

	Matrix& Matrix::operator=(Matrix&& mat) noexcept
	{
		row_ = mat.row_;
		column_ = mat.column_;
		data_ = std::move(mat.data_);
		return *this;
	}

	Matrix operator* (Matrix mat, float f)
	{
		for (unsigned long i = 0; i < mat.row_ * mat.column_; i++)
			mat.data_[i] *= f;
		return mat;
	}

	Matrix operator* (Matrix lhs, Matrix rhs)
	{
#ifndef DKVR_MATH_MAT_ARITHMETIC_NOTHROW
		if (lhs.column_ != rhs.row_)
			throw std::invalid_argument("lhs column length and rhs row length are different.");
#endif
		unsigned long m = lhs.row_;
		unsigned long n = lhs.column_;
		unsigned long q = rhs.column_;

		Matrix result(m, q);
		result.Fill(0);
		for (unsigned long i = 0; i < m; i++)
			for (unsigned long j = 0; j < q; j++)
				for (unsigned long k = 0; k < n; k++)
					result[i][j] += lhs[i][k] * rhs[k][j];
		return result;
	}

	Vector operator*(Matrix mat, Vector vec)
	{
#	ifndef DKVR_MATH_MAT_ARITHMETIC_NOTHROW
		if (mat.column_ != vec.size())
			throw std::invalid_argument("lhs matrix column length and rhs vector length are different.");
#	endif

		Vector result(mat.row_);
		result.Fill(0);
		for (unsigned long i = 0; i < mat.row_; i++)
			for (unsigned long j = 0; j < mat.column_; j++)
				result[i] += mat[i][j] * vec[j];
		return result;
	}

	Matrix operator*(Vector vec, Matrix mat)
	{
#	ifndef DKVR_MATH_MAT_ARITHMETIC_NOTHROW
		if (vec.size() != mat.row_)
			throw std::invalid_argument("lhs vector length and rhs matrix column length are different.");
#	endif
		Matrix result(1, mat.column_);
		result.Fill(0);
		for (unsigned long i = 0; i < result.column_; i++)
			for (unsigned long j = 0; j < vec.size(); j++)
				result.data_[i] = vec[j] * mat[j][i];
		return result;
	}

}	// namespace dkvr

