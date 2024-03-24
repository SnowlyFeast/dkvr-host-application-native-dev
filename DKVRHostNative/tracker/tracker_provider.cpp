#include "tracker_provider.h"

#include <algorithm>

namespace dkvr {

	TrackerProvider& TrackerProvider::GetInstance()
	{
		static TrackerProvider instance;
		return instance;
	}

	TrackerProvider::TrackerProvider()
	{
	}

	TrackerProvider::~TrackerProvider()
	{
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

#define DKVR_DEBUG_TRACKER_CONNECTION_DETAIL
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