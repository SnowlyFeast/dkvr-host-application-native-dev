#pragma once

namespace dkvr {

	struct Quaternion
	{
		constexpr float& operator[] (int i) { return (i % 2 ? (i == 1 ? x : z) : (i == 0 ? w : y)); }

		float w, x, y, z;
	};

}	// namespace dkvr

