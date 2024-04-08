#pragma once

namespace dkvr {

	struct Quaternion
	{
		constexpr float& operator[] (int i) { return (i % 2 ? (i == 1 ? y : w) : (i == 0 ? x : z)); }

		float x, y, z, w;
	};

}	// namespace dkvr

