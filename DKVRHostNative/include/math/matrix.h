#pragma once

#ifdef DKVR_MAT_NOTHROW
#	define DKVR_MATH_MAT_SIZE_LIMIT_NOTHROW
#	define DKVR_MATH_MAT_ARITHMETIC_NOTHROW
#endif

#include <vector>

#include "math/vector.h"

namespace dkvr {

	class Matrix
	{
	public:
		static Matrix CreateIdentity(unsigned long size);
		static Matrix CreateGramMatrix(Vector vec);

		Matrix() : row_(0), column_(0), data_() { }
		Matrix(unsigned long row, unsigned long column) : row_(row), column_(column), data_(row_* column_, 0.0f) { }
		Matrix(const Matrix& mat) : row_(mat.row_), column_(mat.column_), data_(mat.data_) { }
		Matrix(Matrix&& mat) noexcept : row_(mat.row_), column_(mat.column_), data_(std::move(mat.data_)) { }
		Matrix(const Vector& vec);

		void Fill(float values = 0.0f);
		Vector GetColumnVector(unsigned long c) const;
		Matrix GetTranspose() const;
		Matrix GetMinor(unsigned long r, unsigned long c) const;
		float GetDeterminant() const;
		Matrix GetInverse() const;

		Matrix& operator= (const Matrix& mat);
		Matrix& operator= (Matrix&& mat) noexcept;

		// no bound check indexer
		float* operator[] (unsigned long i) { return data_.data() + i * column_; }
		const float* operator[] (unsigned long i) const { return data_.data() + i * column_; }

		friend Matrix operator* (Matrix mat, float f);
		friend Matrix operator* (float f, Matrix mat) { return mat * f; }
		friend Matrix operator* (Matrix lhs, Matrix rhs);
		friend Vector operator* (Matrix mat, Vector vec);
		friend Matrix operator* (Vector vec, Matrix mat);	// vec is considered transposed in this op

		unsigned long row() const { return row_; }
		unsigned long column() const { return column_; }
		const float* data() const { return data_.data(); }
		size_t size() const { return data_.size(); }

	protected:
		unsigned long row_;
		unsigned long column_;
		std::vector<float> data_;
	};

}	// namespace dkvr