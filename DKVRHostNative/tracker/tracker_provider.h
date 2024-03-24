#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include "tracker.h"
#include "atomic_tracker.h"

#include "util/logger.h"

namespace dkvr {

	class TrackerProvider
	{
	public:
		static TrackerProvider& GetInstance();

		AtomicTracker FindExistOrInsertNew(unsigned long address);
		std::vector<AtomicTracker> GetAllTrackers();

	private:
		using TrackerMutexPair = std::pair<Tracker, std::shared_ptr<std::mutex>>;

		TrackerProvider();
		~TrackerProvider();
		TrackerProvider(const TrackerProvider&) = delete;
		TrackerProvider(TrackerProvider&&) = delete;
		void operator= (const TrackerProvider&) = delete;
		void operator= (TrackerProvider&&) = delete;

		AtomicTracker InternalAddTracker(unsigned long address);

		mutable std::mutex mutex_;
		std::vector<TrackerMutexPair> trackers_;

		Logger& logger_ = Logger::GetInstance();

	};

}	// namespace dkvr