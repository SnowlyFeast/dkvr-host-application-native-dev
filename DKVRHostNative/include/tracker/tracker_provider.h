#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include "tracker/tracker.h"
#include "tracker/atomic_tracker.h"
#include "util/logger.h"

namespace dkvr {

	class TrackerProvider
	{
	public:
		TrackerProvider() : mutex_(), trackers_() { }
		~TrackerProvider();

		AtomicTracker FindExistOrInsertNew(unsigned long address);
		AtomicTracker FindByIndex(int index);
		ConstAtomicTracker FindByIndex(int index) const;
		AtomicTracker FindByName(std::string name);
		std::vector<AtomicTracker> GetAllTrackers();


		int GetCount() const;
		int GetIndexOf(const Tracker* target);

	private:
		using TrackerMutexPair = std::pair<Tracker, std::shared_ptr<std::mutex>>;

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