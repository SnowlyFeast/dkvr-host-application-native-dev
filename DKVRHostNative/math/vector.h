#pragma once

#include <vector>

namespace dkvr {

	class Vector
	{
	public:
		Vector() : data_() { }
		Vector(size_t size) : data_(size, 0.0f) { }
		Vector(size_t size, float arg...);
		Vector(const std::vector<float> std_vec) : data_(std_vec) { }
		Vector(std::vector<float>&& std_vec) noexcept : data_(std_vec) { }
		Vector(const Vector& vec) : data_(vec.data_) { }
		Vector(Vector&& vec) noexcept : data_(std::move(vec.data_)) { }

		void Fill(float value = 0.0f);
		float GetEuclidianNorm(bool sqrt = true) const;
		Vector GetNormalized() const;

		Vector& operator= (const Vector& vec);
		Vector& operator= (Vector&& vec) noexcept;

		// no bound check indexer
		float& operator[] (size_t i) { return data_[i]; };
		float operator[] (size_t i) const { return data_[i]; }

		friend Vector operator* (Vector v, float d);
		friend Vector operator* (float d, Vector v) { return v * d; }
		friend float operator* (Vector lhs, Vector rhs);	// it's dot-product, not element-wise multiplication
		friend Vector operator+ (Vector lhs, Vector rhs);
		friend Vector operator- (Vector lhs, Vector rhs);

		size_t size() const { return data_.size(); }
		float* data() { return data_.data(); }
		const float* data() const { return data_.data(); }
		
	protected:
		std::vector<float> data_;
	};

	struct Vector3
	{
		Vector ToVector() const { return Vector(3, x, y, z); }

		float& operator[] (int i) { return i == 0 ? x : (i == 1 ? y : z); }
		float operator[] (int i) const { return i == 0 ? x : (i == 1 ? y : z); }
		Vector3& operator+= (Vector3 rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
		Vector3& operator*= (float f) { x *= f; y *= f; z *= f; return *this; }
		Vector3& operator/= (float f) { x /= f; y /= f; z /= f; return *this; }

		friend Vector3 operator+ (Vector3 lhs, Vector3 rhs) { return Vector3{ lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z }; }
		friend Vector3 operator- (Vector3 lhs, Vector3 rhs) { return Vector3{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z }; }

		float x, y, z;
	};

}	// namespace dkvr
