#include "export/dkvr_host.h"

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include "calibrator/calibration_manager.h"
#include "controller/instruction_dispatcher.h"
#include "controller/tracker_updater.h"
#include "network/network_service.h"
#include "tracker/tracker_provider.h"
#include "util/logger.h"

#define DKVR_HOST_VERSION	1001

namespace dkvr {

	class DKVRHost
	{
	public:
		DKVRHost();

		void Run();
		void Stop();
		bool IsRunning() const { return is_running_; }

		// tracker
		int GetTrackerCount() const { return tk_provider_.GetCount(); }
		unsigned long GetTrackerIPAdress(int index) const { return FindTrackerAndGet(index, &Tracker::address); }
		std::string GetTrackerName(int index) const { return FindTrackerAndGet(index, &Tracker::name, std::string("")); }
		ConnectionStatus GetTrackerConnectionStatus(int index) const { return FindTrackerAndGet(index, &Tracker::connection_status); }
		long long GetTrackerRtt(int index) const { return FindTrackerAndGet(index, &Tracker::rtt, -1LL); }
		bool GetTrackerAcitve(int index) const { return FindTrackerAndGet(index, &Tracker::active); }
		bool GetTrackerRaw(int index) const { return FindTrackerAndGet(index, &Tracker::raw); }
		bool GetTrackerLed(int index) const { return FindTrackerAndGet(index, &Tracker::led); }
		Quaternion GetTrackerQuat(int index) const { return FindTrackerAndGet(index, &Tracker::quaternion, Quaternion{}); }
		Vector3 GetTrackerGyro(int index) const { return FindTrackerAndGet(index, &Tracker::gyro, Vector3{}); }
		Vector3 GetTrackerAccel(int index) const { return FindTrackerAndGet(index, &Tracker::accel, Vector3{}); }
		Vector3 GetTrackerMag(int index) const { return FindTrackerAndGet(index, &Tracker::mag, Vector3{}); }
		int GetTrackerInitResult(int index) const { return FindTrackerAndGet(index, &Tracker::init_result); }
		int GetTrackerLastError(int index) const { return FindTrackerAndGet(index, &Tracker::last_err); }
		int GetTrackerBatteryPerc(int index) const { return FindTrackerAndGet(index, &Tracker::battery_perc); }

		void SetTrackerActive(int index, bool active) { FindTrackerAndSet(index, &Tracker::set_active, active); }
		void SetTrackerRaw(int index, bool raw) { FindTrackerAndSet(index, &Tracker::set_raw, raw); }
		void SetTrackerLed(int index, bool led) { FindTrackerAndSet(index, &Tracker::set_led, led); }

		// calibrator
		CalibrationManager::CalibrationStatus GetCalibratorStatus() { return calib_manager_.GetStatus(); }

	private:
		template <typename T>
		T FindTrackerAndGet(int index, T(Tracker::* getter)(void) const, T not_found = T(0)) const
		{
			ConstAtomicTracker target = tk_provider_.FindByIndex(index);
			return target ? (target->*getter)() : not_found;
		}

		template <typename T>
		void FindTrackerAndSet(int index, void(Tracker::* setter)(T), const T arg)
		{
			AtomicTracker target = tk_provider_.FindByIndex(index);
			if (target)
				(target->*setter)(arg);
		}

		NetworkService net_service_;
		TrackerProvider tk_provider_;
		InstructionDispatcher inst_dispatcher_;
		TrackerUpdater tracker_updater_;
		CalibrationManager calib_manager_;
		Logger& logger_ = Logger::GetInstance();

		bool is_running_ = false;
	};

	DKVRHost::DKVRHost() try :
		net_service_(),
		tk_provider_(),
		inst_dispatcher_(net_service_, tk_provider_),
		tracker_updater_(net_service_, tk_provider_),
		calib_manager_(tk_provider_) { }
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

}


#define DKVRHOST(handle)	(static_cast<dkvr::DKVRHost*>(handle))

int dkvrVersion() { return DKVR_HOST_VERSION; }
int dkvrAssertVersion(int version) { return DKVR_HOST_VERSION == version; }

void dkvrCreateInstance(DKVRHostHandle handle) { try { handle = new dkvr::DKVRHost(); } catch (std::exception except) { handle = nullptr; } }
void dkvrDeleteInstance(DKVRHostHandle handle) { delete handle; handle = nullptr; }
void dkvrRunHost(DKVRHostHandle handle) { DKVRHOST(handle)->Run(); }
void dkvrIsRunning(DKVRHostHandle handle, int* result) { *result = DKVRHOST(handle)->IsRunning(); }
void dkvrStopHost(DKVRHostHandle handle) { DKVRHOST(handle)->Stop(); }

void dkvrTrackerCount(DKVRHostHandle handle, int* out) { *out = DKVRHOST(handle)->GetTrackerCount(); }
void dkvrTrackerAddress(DKVRHostHandle handle, int index, long* out) { *out = DKVRHOST(handle)->GetTrackerIPAdress(index); }
void dkvrTrackerName(DKVRHostHandle handle, int index, char* out, int len)
{
	std::string name = DKVRHOST(handle)->GetTrackerName(index);
	memcpy_s(out, len, name.c_str(), name.size());
}
void dkvrTrackerConnectionStatus(DKVRHostHandle handle, int index, int* out) { *out = static_cast<int>(DKVRHOST(handle)->GetTrackerConnectionStatus(index)); }

void dkvrTrackerRtt(DKVRHostHandle handle, int index, int* out) { *out = static_cast<int>(DKVRHOST(handle)->GetTrackerRtt(index)); }
void dkvrTrackerAcitve(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerAcitve(index); }
void dkvrTrackerRaw(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerRaw(index); }
void dkvrTrackerLed(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerLed(index); }
void dkvrTrackerQuat(DKVRHostHandle handle, int index, float* out)
{
	dkvr::Quaternion quat = DKVRHOST(handle)->GetTrackerQuat(index);
	memcpy_s(out, sizeof(quat), &quat, sizeof(quat));
}
void dkvrTrackerGyro(DKVRHostHandle handle, int index, float* out)
{
	dkvr::Vector3 vec = DKVRHOST(handle)->GetTrackerGyro(index);
	memcpy_s(out, sizeof(vec), &vec, sizeof(vec));
}
void dkvrTrackerAccel(DKVRHostHandle handle, int index, float* out) 
{
	dkvr::Vector3 vec = DKVRHOST(handle)->GetTrackerAccel(index);
	memcpy_s(out, sizeof(vec), &vec, sizeof(vec));
}
void dkvrTrackerMag(DKVRHostHandle handle, int index, float* out)
{
	dkvr::Vector3 vec = DKVRHOST(handle)->GetTrackerMag(index);
	memcpy_s(out, sizeof(vec), &vec, sizeof(vec));
}
void dkvrTrackerInitResult(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerInitResult(index); }
void dkvrTrackerLastError(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerLastError(index); }
void dkvrTrackerBatteryPerc(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerBatteryPerc(index); }

void dkvrTrackerSetActive(DKVRHostHandle handle, int index, int in) { DKVRHOST(handle)->SetTrackerActive(index, in); }
void dkvrTrackerSetRaw(DKVRHostHandle handle, int index, int in) { DKVRHOST(handle)->SetTrackerRaw(index, in); }
void dkvrTrackerSetLed(DKVRHostHandle handle, int index, int in) { DKVRHOST(handle)->SetTrackerLed(index, in); }
	