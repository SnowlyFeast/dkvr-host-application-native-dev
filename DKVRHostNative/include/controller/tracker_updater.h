#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "network/network_service.h"
#include "tracker/tracker_provider.h"
#include "util/logger.h"

namespace dkvr {

	class TrackerUpdater
	{
	public:
		TrackerUpdater(NetworkService& net_service, TrackerProvider& tk_provider);

		void Run();
		void Stop();

	private:
		void UpdaterThreadLoop();
		void UpdateTracker();

		void UpdateConnection(Tracker* target);
		void UpdateHeartbeat(Tracker* target);
		void UpdateRtt(Tracker* target);
		void MatchConfigurationWithClient(Tracker* target);

		std::unique_ptr<std::thread> thread_ptr_;
		std::atomic_bool exit_flag_;
		std::chrono::steady_clock::time_point now_;

		NetworkService& net_service_;
		TrackerProvider& tk_provider_;
		Logger& logger_ = Logger::GetInstance();
	};

}	// namespace dkvr