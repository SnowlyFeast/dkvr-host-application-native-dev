#pragma once

#include <cstdint>

namespace dkvr {

	struct TrackerStatus
	{
	public:
		uint8_t init_result;
		uint8_t last_err;
		uint8_t battery_perc;
	};

}	// namespace dkvr