#pragma once

#include <cstdint>
#include <string>
#include <type_traits>

#include "instruction/instruction_format.h"

namespace dkvr
{
	namespace
	{
		constexpr size_t kMessageLength = sizeof(Instruction::payload) - sizeof(uint32_t) - sizeof(uint8_t);
	}

	struct TrackerDebug
	{
		uint32_t timestamp;
		uint8_t dkvr_err;
		char msg[kMessageLength];

		std::string ToString() const { return std::string(msg); }
	};

	static_assert(std::is_trivial_v<TrackerDebug>);
	static_assert(std::is_standard_layout_v<TrackerDebug>);

}	// namespace dkvr