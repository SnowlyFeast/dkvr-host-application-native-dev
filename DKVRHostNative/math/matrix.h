#pragma once

#ifdef DKVR_MAT_NOTHROW
#	define DKVR_MATH_MAT_SIZE_LIMIT_NOTHROW
#	define DKVR_MATH_MAT_ARITHMETIC_NOTHROW
#endif

#include <algorithm>

#include "vector.h"

namespace dkvr {

	class Matrix
	{
	public:
		static Matrix CreateIdentity(unsigned long size);
		static Matrix CreateGramMatrix(Vector vec);

		Matrix() : row_(0), column_(0), data_(nullptr) { }
		Matrix(unsigned long row, unsigned long column);
		Matrix(const Matrix& mat);
		Matrix(Matrix&& mat) noexcept;
		virtual ~Matrix();

		void Fill(float def = 0) { std::fill(data_, data_ + row_ * column_, def); }
		Vector GetColumnVector(unsigned long c) const;
		Matrix GetTranspose() const;
		Matrix GetMinor(unsigned long r, unsigned long c) const;
		float GetDeterminant() const;
		Matrix GetInverse() const;

		Matrix& operator= (Matrix mat) noexcept;
		Matrix& operator= (Vector vec) noexcept;

		// no bound check indexer
		float* operator[] (unsigned long i) const { return &data_[i * column_]; }

		friend Matrix operator* (Matrix mat, float f);
		friend Matrix operator* (float f, Matrix mat) { return mat * f; }
		friend Matrix operator* (Matrix lhs, Matrix rhs);
		friend Vector operator* (Matrix mat, Vector vec);
		// vec is considered transposed in this op
		friend Matrix operator* (Vector vec, Matrix mat);

		friend void swap(Matrix& lhs, Matrix& rhs);

		unsigned long row() const { return row_; }
		unsigned long column() const { return column_; }

	protected:
		unsigned long row_;
		unsigned long column_;
		float* data_;
	};

}	// namespace dkvr