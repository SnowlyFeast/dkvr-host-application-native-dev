#pragma once

#include <cstdint>
#include <vector>

#include "math/vector.h"
#include "math/matrix.h"

namespace dkvr {

	struct TrackerBehavior
	{
		static constexpr uint8_t kInvalid = (0b10000000u);

		static constexpr uint8_t Encode(TrackerBehavior behavior);
		uint8_t Encode() const;
		void Decode(uint8_t behavior);
		void Reset();

		bool active;
		bool raw;
		bool led;
	};

	struct CalibrationMatrix
	{
		void Reset();

		float gyr_transform[12];
		float acc_transform[12];
		float mag_transform[12];

		float gyr_noise_var[3];
		float acc_noise_var[3];
		float mag_noise_var[3];
	};

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
		std::vector<ConfigurationKey> GetEveryInvalid() const;
		bool IsAllValid() const;
		bool IsValid(ConfigurationKey key) const;
		void Validate(ConfigurationKey key) { valid_[static_cast<int>(key)] = true; }
		void Invalidate(ConfigurationKey key) { valid_[static_cast<int>(key)] = false; }
		void ValidateAll();
		void InvalidateAll();
		void InvalidateBehavior() { Invalidate(ConfigurationKey::Behavior); }
		void InvalidateCalibration();

	private:
		bool valid_[static_cast<int>(ConfigurationKey::Size)]{};
	};

}	// namespace dkvr