#pragma once

#include <cstdint>
#include <type_traits>

namespace dkvr {

	union BytePack {
		uint8_t uchar[4];
		uint16_t ushort[2];
		uint32_t ulong;
		float single;
	};

	struct Instruction
	{
		uint8_t opener;
		uint8_t length;
		uint8_t align;
		uint8_t opcode;
		uint32_t sequence;
		BytePack payload[13];
	};

	static_assert(std::is_trivial_v<Instruction>);
	static_assert(std::is_standard_layout_v<Instruction>);

}	// namespace dkvr