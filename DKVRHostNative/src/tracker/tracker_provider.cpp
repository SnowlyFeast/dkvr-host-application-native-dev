#include "tracker/tracker_provider.h"

#include <algorithm>
#include <memory>
#include <mutex>
#include <vector>

#include "tracker/tracker.h"
#include "tracker/atomic_tracker.h"

#define DKVR_DEBUG_TRACKER_CONNECTION_DETAIL	// TOOD: somewhere else

namespace dkvr {

	TrackerProvider& TrackerProvider::GetInstance()
	{
		static TrackerProvider instance;
		return instance;
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

	AtomicTracker TrackerProvider::FindByIndex(int index)
	{
		std::lock_guard<std::mutex> lock(mutex_);

		if (index < 0 || index >= trackers_.size())
			return AtomicTracker();

		TrackerMutexPair& target = trackers_.at(index);
		return AtomicTracker(&target.first, target.second);
	}

	std::vector<AtomicTracker> TrackerProvider::GetAllTrackers()
	{
		std::vector<AtomicTracker> v;
		std::lock_guard<std::mutex> lock(mutex_);
		for (TrackerMutexPair& p : trackers_)
			v.emplace_back(&p.first, p.second);
		return v;
	}

	int TrackerProvider::GetIndexOf(const Tracker* target)
	{
		if (target == nullptr)
			return -1;

		int index = 0;
		std::lock_guard<std::mutex> lock(mutex_);
		for (TrackerMutexPair& pair : trackers_)
		{
			if (pair.first.address() == target->address())
				return index;
			index++;
		}
		return -1;
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