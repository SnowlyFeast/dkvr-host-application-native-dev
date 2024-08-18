#pragma once

#include <cstdint>

namespace dkvr {

	struct TrackerStatus
	{
	public:
		uint8_t init_result;
		uint8_t battery_level;
	};

}	// namespace dkvr