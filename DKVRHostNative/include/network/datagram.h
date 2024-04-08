#pragma once

#include <type_traits>

#include "controller/instruction_format.h"

namespace dkvr {

	struct Datagram
	{
		unsigned long address;
		Instruction buffer;
	};

	static_assert(std::is_trivial_v<Datagram>);

}	// namespace dkvr