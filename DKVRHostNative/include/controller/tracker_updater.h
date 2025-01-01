#pragma once

#include <chrono>

#include "network/network_service.h"
#include "tracker/tracker_provider.h"
#include "util/logger.h"
#include "util/thread_container.h"

namespace dkvr {

	class TrackerUpdater
	{
	public:
		TrackerUpdater(NetworkService& net_service, TrackerProvider& tk_provider);

		void Run();
		void Stop();

	private:
		void UpdateTracker();

		void UpdateConnection(Tracker* target);
		void UpdateHeartbeatAndRtt(Tracker* target);
		void HandleUpdateRequired(Tracker* target);
		void SyncConfigurationWithClient(Tracker* target);
		void UpdateStatusAndStatistic(Tracker* target);

		std::chrono::steady_clock::time_point now_;
		ThreadContainer<TrackerUpdater> updater_thread_;

		NetworkService& net_service_;
		TrackerProvider& tk_provider_;
		Logger& logger_ = Logger::GetInstance();
	};

}	// namespace dkvr