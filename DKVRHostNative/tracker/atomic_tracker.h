#pragma once

#include <mutex>
#include <memory>

#include "tracker.h"

namespace dkvr {

	class AtomicTracker
	{
	public:
		AtomicTracker() : target_(nullptr), lock_() { }
		AtomicTracker(AtomicTracker&& ref) noexcept : target_(ref.target_), lock_(std::move(ref.lock_)) { }
		AtomicTracker(Tracker* tracker, std::shared_ptr<std::mutex> ptr) : target_(tracker), lock_(*ptr) { }

		bool IsNullptr() { return !target_; }

		// [WARN] do not store the reference of Tracker or atomicity will vanish
		Tracker& operator* () { return *target_; }
		Tracker* operator-> () { return target_; }
		operator Tracker* () const { return target_; }

	private:
		AtomicTracker(const AtomicTracker&) = delete;
		void operator=(const AtomicTracker&) = delete;
		void operator=(AtomicTracker&&) = delete;

		Tracker* target_;
		std::unique_lock<std::mutex> lock_;
	};

}	// namespace dkvr