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

	constexpr uint8_t Behavior::Encode() const
	{
		uint8_t result = 0;
		if (active) result |= kBitmaskActive;
		if (raw)	result |= kBitmaskRaw;
		if (led)	result |= kBitmaskLed;
		return result;
	}

	void Behavior::Decode(uint8_t behavior)
	{
		if (behavior & kBitInvalid) return;

		active	= behavior & kBitmaskActive;
		raw		= behavior & kBitmaskRaw;
		led		= behavior & kBitmaskLed;
	}

	void Behavior::Reset()
	{
		active = false;
		raw = false;
		led = true;
	}

	void Calibration::Reset()
	{
		/*
		* identity gyro offset = [ 0  0  0 ]
		* 
		* identity calibration matrix =
		*	[	1.0		0		0		0
		*		0		1.0		0		0
		*		0		0		1.0		0	]
		*/

		std::fill_n(gyro_offset, 3, 0.0f);
		std::fill_n(accel_mat, 12, 0.0f);
		std::fill_n(mag_mat, 12, 0.0f);

		for (int i = 0; i < 3; i++) {
			accel_mat[i * 5] = 1.0f;
			mag_mat[i * 5] = 1.0f;
		}

	}

}	// namespace dkvr