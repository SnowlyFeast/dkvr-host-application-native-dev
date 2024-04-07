#include "vector.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <stdexcept>
#include <vector>

namespace dkvr {

	Vector::Vector(size_t size, float arg ...) : data_(size)
	{
		va_list args;
		va_start(args, arg);
		for (size_t i = 0; i < size; i++) {
			double d = va_arg(args, double);
			data_[i] = static_cast<float>(d);
		}
		va_end(args);
	}

	void Vector::Fill(float value)
	{
		std::fill_n(data_.data(), data_.size(), value);
	}

	float Vector::GetEuclidianNorm(bool sqrt) const
	{
		float result = 0;
		for (float f : data_)
			result += powf(f, 2);
		return sqrt ? std::sqrtf(result) : result;
	}

	Vector Vector::GetNormalized() const
	{
		return *this * (1.0f / GetEuclidianNorm());
	}


	Vector& Vector::operator= (const Vector& vec)
	{
		data_ = std::vector<float>(vec.data_);
		return *this;
	}

	Vector& Vector::operator= (Vector&& vec) noexcept
	{
		data_ = std::move(vec.data_);
		return *this;
	}

	Vector operator*(Vector v, float d)
	{
		for (float& f : v.data_)
			f *= d;
		return v;
	}

	float operator*(Vector lhs, Vector rhs)
	{
#ifndef DKVR_MATH_VEC_ARITHMETIC_NOTHROW
		if (lhs.size() != rhs.size())
			throw std::invalid_argument("vector sizes are differents.");
#endif
		float result = 0;
		for (size_t i = 0; i < lhs.size(); i++)
			result += lhs[i] * rhs[i];
		return result;
	}

	Vector operator+(Vector lhs, Vector rhs)
	{
#ifndef DKVR_MATH_VEC_ARITHMETIC_NOTHROW
		if (lhs.size() != rhs.size())
			throw std::invalid_argument("vector sizes are different.");
#endif
		for (size_t i = 0; i < lhs.size(); i++)
			lhs[i] += rhs[i];
		return lhs;
	}

	Vector operator-(Vector lhs, Vector rhs)
	{
#ifndef DKVR_MATH_VEC_ARITHMETIC_NOTHROW
		if (lhs.size() != rhs.size())
			throw std::invalid_argument("vector sizes are different.");
#endif
		for (size_t i = 0; i < lhs.size(); i++)
			lhs[i] -= rhs[i];
		return lhs;
	}

}	// namespace dkvr