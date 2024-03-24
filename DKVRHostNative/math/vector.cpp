#include "vector.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <stdexcept>

namespace dkvr {

	Vector::Vector(size_t size) : size_(size), data_(size ? new float[size] : nullptr) { }

	Vector::Vector(size_t size, float arg ...) : Vector(size)
	{
		va_list args;
		va_start(args, arg);
		for (size_t i = 0; i < size; i++) {
			double d = va_arg(args, double);
			data_[i] = static_cast<float>(d);
		}
		va_end(args);
	}

	Vector::Vector(const Vector& vec) : Vector(vec.size_)
	{
		std::copy_n(vec.data_, size_, data_);
	}

	Vector::Vector(Vector&& vec) noexcept : Vector()
	{
		swap(*this, vec);
	}

	Vector::~Vector()
	{
		delete[] data_;
	}

	void Vector::Fill(float def)
	{
		std::fill_n(data_, size_, def);
	}

	float Vector::GetEuclidianNorm(bool sqrt) const
	{
		float result = 0;
		for (size_t i = 0; i < size_; i++)
			result += powf(data_[i], 2);
		return sqrt ? std::sqrtf(result) : result;
	}

	Vector Vector::GetNormalized() const
	{
		return *this * (1.0f / GetEuclidianNorm());
	}

	Vector& Vector::operator=(Vector vec) noexcept
	{
		swap(*this, vec);
		return *this;
	}

	Vector operator*(Vector v, float d)
	{
		for (size_t i = 0; i < v.size_; i++)
			v[i] *= d;
		return v;
	}

	float operator*(Vector lhs, Vector rhs)
	{
#ifndef DKVR_MATH_VEC_ARITHMETIC_NOTHROW
		if (lhs.size_ != rhs.size_)
			throw std::invalid_argument("vector sizes are differents.");
#endif
		float result = 0;
		for (size_t i = 0; i < lhs.size_; i++)
			result += lhs[i] * rhs[i];
		return result;
	}

	Vector operator+(Vector lhs, Vector rhs)
	{
#ifndef DKVR_MATH_VEC_ARITHMETIC_NOTHROW
		if (lhs.size_ != rhs.size_)
			throw std::invalid_argument("vector sizes are different.");
#endif
		for (size_t i = 0; i < lhs.size_; i++)
			lhs[i] += rhs[i];
		return lhs;
	}

	Vector operator-(Vector lhs, Vector rhs)
	{
#ifndef DKVR_MATH_VEC_ARITHMETIC_NOTHROW
		if (lhs.size_ != rhs.size_)
			throw std::invalid_argument("vector sizes are different.");
#endif
		for (size_t i = 0; i < lhs.size_; i++)
			lhs[i] -= rhs[i];
		return lhs;
	}

	void swap(Vector& lhs, Vector& rhs)
	{
		std::swap(lhs.size_, rhs.size_);
		std::swap(lhs.data_, rhs.data_);
	}

}	// namespace dkvr