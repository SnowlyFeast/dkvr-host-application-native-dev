#include "tracker/tracker_config_validator.h"

#include <vector>

namespace dkvr {

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

}	// namespace dkvr