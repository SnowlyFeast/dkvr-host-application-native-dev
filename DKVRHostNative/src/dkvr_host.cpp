#include "dkvr_host.h"

#include <stdexcept>

namespace dkvr {

	DKVRHost::DKVRHost() try :
		net_service_(),
		tk_provider_(),
		inst_dispatcher_(net_service_, tk_provider_),
		tracker_updater_(net_service_, tk_provider_),
		calib_manager_(tk_provider_)
	{ }
	catch (std::runtime_error except)
	{
		throw except;	// just rethrow it
	}

	void DKVRHost::Run()
	{
		if (is_running_) return;

		if (net_service_.Run())	return;
		inst_dispatcher_.Run();
		tracker_updater_.Run();

		is_running_ = true;
	}

	void DKVRHost::Stop()
	{
		// stop service on reverse order
		tracker_updater_.Stop();
		inst_dispatcher_.Stop();
		net_service_.Stop();

		is_running_ = false;
	}

	bool DKVRHost::IsRunning() const
	{
		return is_running_;
	}

}
