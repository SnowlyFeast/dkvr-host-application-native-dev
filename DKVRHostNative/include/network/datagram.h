#pragma once

#include <type_traits>

#include "instruction/instruction_format.h"

namespace dkvr {

	struct Datagram
	{
		unsigned long address;
		Instruction buffer;
	};

	static_assert(std::is_trivial_v<Datagram>);
	static_assert(std::is_standard_layout_v<Datagram>);

}	// namespace dkvr