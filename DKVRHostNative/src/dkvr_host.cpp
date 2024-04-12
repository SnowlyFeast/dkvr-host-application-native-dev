#include "export/dkvr_host.h"

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <sstream>
#include <string>

#ifdef _DEBUG
#else
#	define DKVR_LOGGER_GLOBAL_LEVEL		1
#endif

#include "calibrator/calibration_manager.h"
#include "controller/instruction_dispatcher.h"
#include "controller/tracker_updater.h"
#include "network/network_service.h"
#include "tracker/tracker_provider.h"
#include "util/logger.h"
#include "version.h"

#define DKVRHOST(handle)	(static_cast<dkvr::DKVRHost*>(handle))

namespace dkvr {

	class DKVRHost
	{
	public:
		DKVRHost();

		// instance control
		void Run();
		void Stop();
		bool IsRunning() const { return is_running_; }

		// logger
		void SetLoggerLevelDebug() { logger_.set_level(dkvr::Logger::Level::Debug); }
		void SetLoggerLevelInfo() { logger_.set_level(dkvr::Logger::Level::Info); }
		void SetLoggerLevelError() { logger_.set_level(dkvr::Logger::Level::Error); }

		int GetUncheckedLogCount() const { return logger_.GetUncheckedCount(); }
		std::string GetUncheckedLogOne() { logger_output_.str(std::string()); logger_output_.clear(); logger_.PrintUnchecked(1); return logger_output_.str(); }
		std::string GetUncheckedLogAll() { logger_output_.str(std::string()); logger_output_.clear(); logger_.PrintUnchecked(); return logger_output_.str(); }

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
		int GetCalibratorTarget() { return calib_manager_.GetCurrentCalibrationTarget(); }
		void BeginCalibrationWith(int index) { calib_manager_.Begin(index); }
		void AbortCalibration() { calib_manager_.Abort(); }

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
		std::stringstream logger_output_;
	};

	DKVRHost::DKVRHost() try :
		net_service_(),
		tk_provider_(),
		inst_dispatcher_(net_service_, tk_provider_),
		tracker_updater_(net_service_, tk_provider_),
		calib_manager_(tk_provider_)
	{
		logger_.set_level(
#ifdef _DEBUG
			dkvr::Logger::Level::Debug
#else
			dkvr::Logger::Level::Info
#endif
		);
		logger_.set_mode(dkvr::Logger::Mode::Burst);
		logger_.set_ostream(logger_output_);
	}
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

// version
void dkvrVersion(int* out) { *out = DKVR_HOST_VERSION; }
void dkvrAssertVersion(int version, int* success) { *success = (DKVR_HOST_VERSION == version); }

// instance control
void dkvrCreateInstance(DKVRHostHandle* hptr) { try { *hptr = new dkvr::DKVRHost(); } catch (std::exception except) { *hptr = nullptr; } }
void dkvrDeleteInstance(DKVRHostHandle* hptr) { delete *hptr; *hptr = nullptr; }
void dkvrRunHost(DKVRHostHandle handle) { DKVRHOST(handle)->Run(); }
void dkvrStopHost(DKVRHostHandle handle) { DKVRHOST(handle)->Stop(); }
void dkvrIsRunning(DKVRHostHandle handle, int* running) { *running = (DKVRHOST(handle)->IsRunning()); }

// logger
void dkvrLoggerSetLevelDebug(DKVRHostHandle handle) { DKVRHOST(handle)->SetLoggerLevelDebug(); }
void dkvrLoggerSetLevelInfo(DKVRHostHandle handle) { DKVRHOST(handle)->SetLoggerLevelInfo(); }
void dkvrLoggerSetLevelError(DKVRHostHandle handle) { DKVRHOST(handle)->SetLoggerLevelError(); }

void dkvrLoggerGetUncheckCount(DKVRHostHandle handle, int* out) { *out = DKVRHOST(handle)->GetUncheckedLogCount(); }
void dkvrLoggerGetUncheckedLogOne(DKVRHostHandle handle, char* out, int len)
{
	std::string log = DKVRHOST(handle)->GetUncheckedLogOne();
	memcpy_s(out, len, log.c_str(), log.size() + 1);
}
void dkvrLoggerGetUncheckedLogAll(DKVRHostHandle handle, char* out, int len)
{
	std::string log = DKVRHOST(handle)->GetUncheckedLogAll();
	memcpy_s(out, len, log.c_str(), log.size() + 1);
}

// tracker
void dkvrTrackerGetCount(DKVRHostHandle handle, int* out) { *out = DKVRHOST(handle)->GetTrackerCount(); }
void dkvrTrackerGetAddress(DKVRHostHandle handle, int index, long* out) { *out = DKVRHOST(handle)->GetTrackerIPAdress(index); }
void dkvrTrackerGetName(DKVRHostHandle handle, int index, char* out, int len)
{
	std::string name = DKVRHOST(handle)->GetTrackerName(index);
	memcpy_s(out, len, name.c_str(), name.size() + 1);
}
void dkvrTrackerGetConnectionStatus(DKVRHostHandle handle, int index, int* out)
{
	*out = static_cast<int>(DKVRHOST(handle)->GetTrackerConnectionStatus(index));
}
void dkvrTrackerGetRtt(DKVRHostHandle handle, int index, int* out) { *out = static_cast<int>(DKVRHOST(handle)->GetTrackerRtt(index)); }
void dkvrTrackerGetAcitve(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerAcitve(index); }
void dkvrTrackerGetRaw(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerRaw(index); }
void dkvrTrackerGetLed(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerLed(index); }
void dkvrTrackerGetQuat(DKVRHostHandle handle, int index, float* out)
{
	dkvr::Quaternion quat = DKVRHOST(handle)->GetTrackerQuat(index);
	memcpy_s(out, sizeof(quat), &quat, sizeof(quat));
}
void dkvrTrackerGetGyro(DKVRHostHandle handle, int index, float* out)
{
	dkvr::Vector3 vec = DKVRHOST(handle)->GetTrackerGyro(index);
	memcpy_s(out, sizeof(vec), &vec, sizeof(vec));
}
void dkvrTrackerGetAccel(DKVRHostHandle handle, int index, float* out)
{
	dkvr::Vector3 vec = DKVRHOST(handle)->GetTrackerAccel(index);
	memcpy_s(out, sizeof(vec), &vec, sizeof(vec));
}
void dkvrTrackerGetMag(DKVRHostHandle handle, int index, float* out)
{
	dkvr::Vector3 vec = DKVRHOST(handle)->GetTrackerMag(index);
	memcpy_s(out, sizeof(vec), &vec, sizeof(vec));
}
void dkvrTrackerGetInitResult(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerInitResult(index); }
void dkvrTrackerGetLastError(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerLastError(index); }
void dkvrTrackerGetBatteryPerc(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerBatteryPerc(index); }

void dkvrTrackerSetActive(DKVRHostHandle handle, int index, int in) { DKVRHOST(handle)->SetTrackerActive(index, in); }
void dkvrTrackerSetRaw(DKVRHostHandle handle, int index, int in) { DKVRHOST(handle)->SetTrackerRaw(index, in); }
void dkvrTrackerSetLed(DKVRHostHandle handle, int index, int in) { DKVRHOST(handle)->SetTrackerLed(index, in); }

// calibrator
void dkvrCalibratorGetStatus(DKVRHostHandle handle, int* out) { *out = static_cast<int>(DKVRHOST(handle)->GetCalibratorStatus()); }
void dkvrCalibratorGetCurrentTarget(DKVRHostHandle handle, int* out) { *out = DKVRHOST(handle)->GetCalibratorTarget(); }
void dkvrCalibratorBeginWith(DKVRHostHandle handle, int index) { DKVRHOST(handle)->BeginCalibrationWith(index); }
void dkvrCalibratorAbort(DKVRHostHandle handle) { DKVRHOST(handle)->AbortCalibration(); }