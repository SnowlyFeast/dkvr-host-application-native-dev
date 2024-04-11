#include "tracker/tracker_provider.h"

#include <algorithm>
#include <memory>
#include <mutex>
#include <vector>

#include "tracker/tracker.h"
#include "tracker/atomic_tracker.h"

#define DKVR_DEBUG_TRACKER_CONNECTION_DETAIL	// TOOD: somewhere else

namespace dkvr {

	TrackerProvider::~TrackerProvider()
	{
		for (TrackerMutexPair& p : trackers_)
		{
			std::unique_lock<std::mutex> lock(*p.second);
		}
	}

	AtomicTracker TrackerProvider::FindExistOrInsertNew(unsigned long address)
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			auto iter = std::find_if(
				trackers_.begin(),
				trackers_.end(),
				[address](const TrackerMutexPair& item) {
					return item.first.address() == address;
				}
			);
			if (iter != trackers_.end())
				return AtomicTracker(&iter->first, iter->second);
		}
		return InternalAddTracker(address);
	}

	std::vector<AtomicTracker> TrackerProvider::GetAllTrackers()
	{
		std::vector<AtomicTracker> v;
		for (TrackerMutexPair& p : trackers_)
			v.emplace_back(&p.first, p.second);
		return v;
	}

	AtomicTracker TrackerProvider::InternalAddTracker(unsigned long address)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		trackers_.emplace_back(Tracker(address), std::make_shared<std::mutex>());
		TrackerMutexPair& last = trackers_.back();
#ifdef DKVR_DEBUG_TRACKER_CONNECTION_DETAIL
		unsigned char* ptr = reinterpret_cast<unsigned char*>(&address);
		logger_.Debug("Tracker added with ip {:d}.{:d}.{:d}.{:d}", ptr[0], ptr[1], ptr[2], ptr[3]);
#endif
		return AtomicTracker(&last.first, last.second);
	}

}	// namespace dkvr