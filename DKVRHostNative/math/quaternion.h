#pragma once

#include <type_traits>

namespace dkvr {

	struct Quaternion
	{
		constexpr float& operator[] (int i) { return (i % 2 ? (i == 1 ? y : w) : (i == 0 ? x : z)); }

		float x, y, z, w;
	};

	static_assert(std::is_trivial_v<Quaternion>);

}	// namespace dkvr

