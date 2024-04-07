#pragma once

#include <type_traits>

namespace dkvr {

	class Vector
	{
	public:
		Vector() : size_(0), data_(nullptr) { }
		Vector(size_t size);
		Vector(size_t size, float arg...);
		Vector(const Vector& vec);
		Vector(Vector&& vec) noexcept;
		virtual ~Vector();

		void Fill(float def = 0);
		float GetEuclidianNorm(bool sqrt = true) const;
		Vector GetNormalized() const;

		Vector& operator= (Vector vec) noexcept;

		// no bound check indexer
		float& operator[] (size_t i) { return data_[i]; };
		float operator[] (size_t i) const { return data_[i]; }

		friend Vector operator* (Vector v, float d);
		friend Vector operator* (float d, Vector v) { return v * d; }
		// it's dot-product, not element-wise multiplication
		friend float operator* (Vector lhs, Vector rhs);
		friend Vector operator+ (Vector lhs, Vector rhs);
		friend Vector operator- (Vector lhs, Vector rhs);

		friend void swap(Vector& lhs, Vector& rhs);

		size_t size() const { return size_; }
		float* data() const { return data_; }

	protected:
		size_t size_;
		float* data_;
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

	static_assert(std::is_standard_layout_v<Vector3>);

}	// namespace dkvr
