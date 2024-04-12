#pragma once

#include <mutex>
#include <memory>

#include "tracker/tracker.h"

namespace dkvr {

	template<class T, class U>
	concept Dereived = std::is_base_of<T, U>::value;

	template<Dereived<Tracker> TrackerType>
	class AtomicTrackerBase
	{
	public:
		AtomicTrackerBase() : target_(nullptr), lock_() { }
		AtomicTrackerBase(AtomicTrackerBase&& ref) noexcept : target_(ref.target_), lock_(std::move(ref.lock_)) { }
		AtomicTrackerBase(TrackerType* tracker, std::shared_ptr<std::mutex> ptr) : target_(tracker), lock_(*ptr) { }

		bool IsNullptr() { return !target_; }

		// [WARN] do not store the reference of Tracker or atomicity will vanish
		TrackerType& operator* () { return *target_; }
		TrackerType* operator-> () { return target_; }
		operator TrackerType* () const { return target_; }

	private:
		AtomicTrackerBase(const AtomicTrackerBase&) = delete;
		void operator=(const AtomicTrackerBase&) = delete;
		void operator=(AtomicTrackerBase&&) = delete;

		TrackerType* target_;
		std::unique_lock<std::mutex> lock_;
	};

	using AtomicTracker = AtomicTrackerBase<Tracker>;
	using ConstAtomicTracker = AtomicTrackerBase<const Tracker>;

}	// namespace dkvr