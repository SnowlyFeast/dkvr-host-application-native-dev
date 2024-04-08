#pragma once

#include <vector>

namespace dkvr {

	enum class ConfigurationKey
	{
		Behavior,
		CalibrationGr,
		CalibrationAc,
		CalibrationMg,
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

	private:
		bool valid_[static_cast<int>(ConfigurationKey::Size)]{};
	};

}	// namespace dkvr