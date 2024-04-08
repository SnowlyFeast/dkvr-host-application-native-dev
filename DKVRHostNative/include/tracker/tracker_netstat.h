#pragma once

#include <chrono>
#include <cstdint>

namespace dkvr {

	struct NetworkStatistics
	{
		uint32_t send_sequence_num;
		uint32_t recv_sequence_num;
		std::chrono::steady_clock::time_point last_heartbeat_sent;
		std::chrono::steady_clock::time_point last_heartbeat_recv;
		std::chrono::steady_clock::time_point last_ping_sent;
		std::chrono::milliseconds rtt;
	};

}	// namespace dkvr