#pragma once

#include <thread>
#include <atomic>

#include "network/network_service.h"
#include "tracker/tracker_provider.h"
#include "util/logger.h"

namespace dkvr {

	class TrackerUpdater
	{
	public:
		TrackerUpdater() : thread_(nullptr), exit_flag_(false), now_() { }

		void Run();
		void Stop();

	private:
		void UpdaterThreadLoop();
		void UpdateTracker();

		void UpdateConnection(Tracker* target);
		void UpdateHeartbeat(Tracker* target);
		void UpdateRtt(Tracker* target);
		void MatchConfigurationWithClient(Tracker* target);

		std::thread* thread_;
		std::atomic_bool exit_flag_;
		std::chrono::steady_clock::time_point now_;

		NetworkService& net_service_ = NetworkService::GetInstance();
		TrackerProvider& tk_provider_ = TrackerProvider::GetInstance();
		Logger& logger_ = Logger::GetInstance();
	};

}	// namespace dkvr