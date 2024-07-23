#include "tracker/tracker_configuration.h"

#include <algorithm>
#include <cstdint>
#include <vector>

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

		std::fill_n(gyr_transform, 12, 0.0f);
		std::fill_n(acc_transform, 12, 0.0f);
		std::fill_n(mag_transform, 12, 0.0f);

		for (int i = 0; i < 3; i++) {
			gyr_transform[i * 5] = 1.0f;
			acc_transform[i * 5] = 1.0f;
			mag_transform[i * 5] = 1.0f;
		}

		std::fill_n(gyr_noise_var, 3, 0.0f);
		std::fill_n(acc_noise_var, 3, 0.0f);
		std::fill_n(mag_noise_var, 3, 0.0f);
	}

	std::vector<ConfigurationKey> ConfigurationValidator::GetEveryInvalid() const
	{
		std::vector<ConfigurationKey> vec;
		for (int i = 0; i < sizeof(valid_); i++)
			if (!valid_[i])
				vec.push_back(ConfigurationKey(i));
		return vec;
	}

	bool ConfigurationValidator::IsAllValid() const
	{
		for (bool b : valid_)
			if (!b)
				return false;
		return true;
	}

	bool ConfigurationValidator::IsValid(ConfigurationKey key) const
	{
		return valid_[static_cast<int>(key)];
	}

	void ConfigurationValidator::ValidateAll()
	{
		for (bool& b : valid_)
			b = true;
	}

	void ConfigurationValidator::InvalidateAll()
	{
		for (bool& b : valid_)
			b = false;
	}

	void ConfigurationValidator::InvalidateCalibration()
	{
		Invalidate(ConfigurationKey::GyrTransform);
		Invalidate(ConfigurationKey::AccTransform);
		Invalidate(ConfigurationKey::MagTransform);
		Invalidate(ConfigurationKey::NoiseVariance);
	}

}	// namespace dkvr