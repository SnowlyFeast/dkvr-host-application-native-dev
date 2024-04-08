#include "tracker/tracker_configuration.h"

#include <cstdint>

namespace dkvr {

	constexpr uint8_t kBitmaskActive	= 0b00000001;
	constexpr uint8_t kBitmaskRaw		= 0b00000010;
	constexpr uint8_t kBitmaskLed		= 0b00000100;

	uint8_t Behavior::Encode() const
	{
		uint8_t result = 0;
		if (active) result |= kBitmaskActive;
		if (raw)	result |= kBitmaskRaw;
		if (led)	result |= kBitmaskLed;
		return result;
	}

	void Behavior::Decode(uint8_t behavior)
	{
		active = behavior & kBitmaskActive;
		raw = behavior & kBitmaskRaw;
		led = behavior & kBitmaskLed;
	}

}	// namespace dkvr