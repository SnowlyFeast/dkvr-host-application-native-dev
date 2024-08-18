#pragma once

#include <string>

namespace dkvr {

	enum class ConnectionStatus
	{
		Disconnected,
		Handshaked,
		Connected
	};

	struct TrackerInformation
	{
		unsigned long address;
		std::string name;
		ConnectionStatus connection;
	};

}	// namespace dkvr