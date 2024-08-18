#pragma once

#include <cstdint>

namespace dkvr
{

	struct TrackerStatistic
	{
	public:
		uint8_t execution_time;
		uint8_t interrupt_miss_rate;
		uint8_t imu_miss_rate;
	};
}