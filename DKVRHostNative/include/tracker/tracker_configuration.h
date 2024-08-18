#pragma once

#include <cstdint>
#include <type_traits>
#include <vector>

namespace dkvr {

	struct TrackerBehavior
	{
		static constexpr uint8_t kBitmaskActive = 0x01;
		static constexpr uint8_t kBitmaskRaw = 0x02;
		static constexpr uint8_t kBitmaskLed = 0x04;
		static constexpr uint8_t kBitmaskInvalid = 0x80;

		constexpr uint8_t Encode() const
		{
			uint8_t result = 0;
			if (active)	result |= kBitmaskActive;
			if (raw)    result |= kBitmaskRaw;
			if (led)    result |= kBitmaskLed;
			return result;
		}
		void Decode(uint8_t behavior)
		{
			if (behavior & kBitmaskInvalid) return;

			active = behavior & kBitmaskActive;
			raw = behavior & kBitmaskRaw;
			led = behavior & kBitmaskLed;
		}
		void Reset()
		{
			active = false;
			raw = false;
			led = true;
		}

		bool active;
		bool raw;
		bool led;
	};

	struct TrackerCalibration
	{
		void Reset()
		{
			// column-major
			constexpr float identity[12] = { 1, 0, 0,   0, 1, 0,   0, 0, 1,   0, 0 ,0 };

			std::copy_n(identity, 12, gyr_transform);
			std::copy_n(identity, 12, acc_transform);
			std::copy_n(identity, 12, mag_transform);

			std::fill_n(gyr_noise_var, 3, 0.0f);
			std::fill_n(acc_noise_var, 3, 0.0f);
			std::fill_n(mag_noise_var, 3, 0.0f);
		}

		float gyr_transform[12];
		float acc_transform[12];
		float mag_transform[12];

		float gyr_noise_var[3];
		float acc_noise_var[3];
		float mag_noise_var[3];
	};

	static_assert(std::is_trivial_v<TrackerCalibration>);
	static_assert(std::is_standard_layout_v<TrackerCalibration>);

	enum class ConfigurationKey
	{
		Behavior,
		GyrTransform,
		AccTransform,
		MagTransform,
		NoiseVariance,
		Size	// keep this member at last, not an actual key
	};

	class ConfigurationValidator
	{
	public:
		std::vector<ConfigurationKey> GetEveryInvalid() const
		{
			std::vector<ConfigurationKey> result;
			for (int i = 0; i < sizeof(is_valid_); i++)
				if (!is_valid_[i]) result.push_back(ConfigurationKey(i));
			return result;
		}
		bool IsAllValid() const
		{
			for (bool b : is_valid_)
				if (!b) return false;
			return true;
		}
		bool IsValid(ConfigurationKey key) const { return is_valid_[static_cast<int>(key)]; }

		void Validate(ConfigurationKey key) { is_valid_[static_cast<int>(key)] = true; }
		void Invalidate(ConfigurationKey key) { is_valid_[static_cast<int>(key)] = false; }
		void ValidateAll() { for (bool& b : is_valid_) b = true; }
		void InvalidateAll() { for (bool& b : is_valid_) b = false; }
		void InvalidateBehavior() { Invalidate(ConfigurationKey::Behavior); }
		void InvalidateCalibration()
		{
			Invalidate(ConfigurationKey::GyrTransform);
			Invalidate(ConfigurationKey::AccTransform);
			Invalidate(ConfigurationKey::MagTransform);
			Invalidate(ConfigurationKey::NoiseVariance);
		}

	private:
		bool is_valid_[static_cast<int>(ConfigurationKey::Size)]{};
	};

}	// namespace dkvr