#include "tracker/tracker_configuration.h"

#include <algorithm>
#include <cstdint>

namespace dkvr {

	namespace {
		constexpr uint8_t kBitmaskActive	= 0b00000001;
		constexpr uint8_t kBitmaskRaw		= 0b00000010;
		constexpr uint8_t kBitmaskLed		= 0b00000100;

		constexpr uint8_t kBitInvalid		= 0b10000000;
	}

	constexpr uint8_t TrackerBehavior::Encode(TrackerBehavior behavior)
	{
		uint8_t result = 0;
		if (behavior.active)	result |= kBitmaskActive;
		if (behavior.raw)		result |= kBitmaskRaw;
		if (behavior.led)		result |= kBitmaskLed;
		return result;
	}

	uint8_t TrackerBehavior::Encode() const
	{
		return Encode(*this);
	}

	void TrackerBehavior::Decode(uint8_t behavior)
	{
		if (behavior & kBitInvalid) return;

		active	= behavior & kBitmaskActive;
		raw		= behavior & kBitmaskRaw;
		led		= behavior & kBitmaskLed;
	}

	void TrackerBehavior::Reset()
	{
		active = false;
		raw = false;
		led = true;
	}

	void CalibrationMatrix::Reset()
	{
		/*
		* identity calibration matrix =
		*	[	1		0		0		0
		*		0		1		0		0
		*		0		0		1		0	]
		*/

		std::fill_n(gyr, 12, 0.0f);
		std::fill_n(acc, 12, 0.0f);
		std::fill_n(mag, 12, 0.0f);

		for (int i = 0; i < 3; i++) {
			gyr[i * 5] = 1.0f;
			acc[i * 5] = 1.0f;
			mag[i * 5] = 1.0f;
		}

	}

}	// namespace dkvr