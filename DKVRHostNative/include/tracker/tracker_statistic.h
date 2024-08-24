#pragma once

#include <cstdint>
#include <type_traits>

namespace dkvr
{

	struct TrackerStatistic
	{
	public:
		uint8_t execution_time;
		uint8_t interrupt_miss_rate;
		uint8_t imu_miss_rate;
	};

	static_assert(std::is_trivial_v<TrackerStatistic>);
	static_assert(std::is_standard_layout_v<TrackerStatistic>);

}