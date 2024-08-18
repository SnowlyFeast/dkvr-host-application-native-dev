#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <sstream>
#include <string>

#ifndef _DEBUG
#	define DKVR_LOGGER_GLOBAL_LEVEL		1
#endif

#include "export/dkvr_host.h"

#include "calibrator/calibration_manager.h"
#include "controller/instruction_dispatcher.h"
#include "controller/tracker_updater.h"
#include "network/network_service.h"
#include "tracker/tracker_provider.h"
#include "util/logger.h"
#include "version.h"

#define DKVRHOST(handle)	(static_cast<dkvr::DKVRHost*>(handle))

namespace 
{
	void StringCopy(const std::string& str, char* dst, int len)
	{
		size_t cap = std::min(str.size() + 1, static_cast<size_t>(len));
		memcpy_s(dst, len, str.c_str(), cap);
		dst[cap - 1] = '\0';
	}
}

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
		TrackerCalibration GetTrackerCalibration(int index) const { return FindTrackerAndGet(index, &Tracker::calibration, dkvr::TrackerCalibration{}); }
		Quaternionf GetTrackerOrientation(int index) const { return FindTrackerAndGet(index, &Tracker::orientation, Quaternionf{}); }
		Vector3f GetTrackerGyro(int index) const { return FindTrackerAndGet(index, &Tracker::gyro, Vector3f{}); }
		Vector3f GetTrackerAccel(int index) const { return FindTrackerAndGet(index, &Tracker::accel, Vector3f{}); }
		Vector3f GetTrackerMag(int index) const { return FindTrackerAndGet(index, &Tracker::mag, Vector3f{}); }
		int GetTrackerExecutionTime(int index) const { return FindTrackerAndGet(index, &Tracker::execution_time); }
		int GetTrackerInitResult(int index) const { return FindTrackerAndGet(index, &Tracker::init_result); }
		int GetTrackerBatteryPerc(int index) const { return FindTrackerAndGet(index, &Tracker::battery_perc); }

		void SetTrackerActive(int index, bool active) { FindTrackerAndSet(index, &Tracker::set_active, active); }
		void SetTrackerRaw(int index, bool raw) { FindTrackerAndSet(index, &Tracker::set_raw, raw); }
		void SetTrackerLed(int index, bool led) { FindTrackerAndSet(index, &Tracker::set_led, led); }
		void SetTrackerCalibration(int index, const TrackerCalibration& calib) { FindTrackerAndSet(index, &Tracker::set_calibration, calib); }

		void RequestTrackerStatistic(int index) { FindTrackerAndCall(index, &Tracker::RequestStatisticUpdate); }
		void RequestTrackerStatus(int index) { FindTrackerAndCall(index, &Tracker::RequestStatusUpdate); }
		void RequestTrackerLocate(int index) { FindTrackerAndCall(index, &Tracker::RequestLocate); }

		// calibrator
		CalibrationManager::CalibrationStatus GetCalibratorStatus() { return calib_manager_.GetStatus(); }
		std::string GetCalibratorStatusAsString() { return calib_manager_.GetStatusAsString(); }
		SampleType GetCalibratorRequiredSampleType() { return calib_manager_.GetRequiredSampleType(); }
		std::string GetCalibratorRequiredSampleTypeAsString() { return calib_manager_.GetRequiredSampleTypeAsString(); }
		int GetCalibratorTarget() { return calib_manager_.GetCurrentCalibrationTarget(); }
		void BeginCalibrationWith(int index) { calib_manager_.Begin(index); }
		void AbortCalibration() { calib_manager_.Abort(); }
		void ContinueCalibration() { calib_manager_.Continue(); }

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

		void FindTrackerAndCall(int index, void(Tracker::* callback)(void))
		{
			AtomicTracker target = tk_provider_.FindByIndex(index);
			if (target)
				(target->*callback)();
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
#ifdef _DEBUG
		logger_.set_level(dkvr::Logger::Level::Debug);
#else
		logger_.set_level(dkvr::Logger::Level::Info);
#endif
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

// assertion
static_assert(DKVR_HOST_EXPORTED_HEADER_VER == DKVR_HOST_VERSION);

static_assert(std::is_trivial_v<DKVRVector3>);
static_assert(std::is_standard_layout_v<DKVRVector3>);
static_assert(sizeof DKVRVector3 == sizeof dkvr::Vector3f);

static_assert(std::is_trivial_v<DKVRQuaternion>);
static_assert(std::is_standard_layout_v<DKVRQuaternion>);
static_assert(sizeof DKVRQuaternion == sizeof dkvr::Quaternionf);

static_assert(std::is_trivial_v<DKVRCalibration>);
static_assert(std::is_standard_layout_v<DKVRCalibration>);
static_assert(sizeof DKVRCalibration == sizeof dkvr::TrackerCalibration);
static_assert(offsetof(DKVRCalibration, gyr_transform) == offsetof(dkvr::TrackerCalibration, gyr_transform));
static_assert(offsetof(DKVRCalibration, acc_transform) == offsetof(dkvr::TrackerCalibration, acc_transform));
static_assert(offsetof(DKVRCalibration, mag_transform) == offsetof(dkvr::TrackerCalibration, mag_transform));
static_assert(offsetof(DKVRCalibration, gyr_noise_var) == offsetof(dkvr::TrackerCalibration, gyr_noise_var));
static_assert(offsetof(DKVRCalibration, acc_noise_var) == offsetof(dkvr::TrackerCalibration, acc_noise_var));
static_assert(offsetof(DKVRCalibration, mag_noise_var) == offsetof(dkvr::TrackerCalibration, mag_noise_var));

// version
void __stdcall dkvrVersion(int* out) { *out = DKVR_HOST_VERSION; }
void __stdcall dkvrAssertVersion(int version, int* success) { *success = (DKVR_HOST_VERSION == version); }

// instance control
void __stdcall dkvrCreateInstance(DKVRHostHandle* hptr) { try { *hptr = new dkvr::DKVRHost(); } catch (std::exception except) { *hptr = nullptr; } }
void __stdcall dkvrDeleteInstance(DKVRHostHandle* hptr) { delete *hptr; *hptr = nullptr; }
void __stdcall dkvrRunHost(DKVRHostHandle handle) { DKVRHOST(handle)->Run(); }
void __stdcall dkvrStopHost(DKVRHostHandle handle) { DKVRHOST(handle)->Stop(); }
void __stdcall dkvrIsRunning(DKVRHostHandle handle, int* running) { *running = (DKVRHOST(handle)->IsRunning()); }

// logger
void __stdcall dkvrLoggerSetLevelDebug(DKVRHostHandle handle) { DKVRHOST(handle)->SetLoggerLevelDebug(); }
void __stdcall dkvrLoggerSetLevelInfo(DKVRHostHandle handle) { DKVRHOST(handle)->SetLoggerLevelInfo(); }
void __stdcall dkvrLoggerSetLevelError(DKVRHostHandle handle) { DKVRHOST(handle)->SetLoggerLevelError(); }

void __stdcall dkvrLoggerGetUncheckCount(DKVRHostHandle handle, int* out) { *out = DKVRHOST(handle)->GetUncheckedLogCount(); }
void __stdcall dkvrLoggerGetUncheckedLogOne(DKVRHostHandle handle, char* out, int len)
{
	StringCopy(DKVRHOST(handle)->GetUncheckedLogOne(), out, len);
}
void __stdcall dkvrLoggerGetUncheckedLogAll(DKVRHostHandle handle, char* out, int len)
{
	StringCopy(DKVRHOST(handle)->GetUncheckedLogAll(), out, len);
}

// tracker
void __stdcall dkvrTrackerGetCount(DKVRHostHandle handle, int* out) { *out = DKVRHOST(handle)->GetTrackerCount(); }
void __stdcall dkvrTrackerGetAddress(DKVRHostHandle handle, int index, long* out) { *out = DKVRHOST(handle)->GetTrackerIPAdress(index); }
void __stdcall dkvrTrackerGetName(DKVRHostHandle handle, int index, char* out, int len)
{
	StringCopy(DKVRHOST(handle)->GetTrackerName(index), out, len);
}
void __stdcall dkvrTrackerGetConnectionStatus(DKVRHostHandle handle, int index, int* out) { *out = static_cast<int>(DKVRHOST(handle)->GetTrackerConnectionStatus(index)); }
void __stdcall dkvrTrackerGetRtt(DKVRHostHandle handle, int index, int* out) { *out = static_cast<int>(DKVRHOST(handle)->GetTrackerRtt(index)); }
void __stdcall dkvrTrackerGetActive(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerAcitve(index); }
void __stdcall dkvrTrackerGetRaw(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerRaw(index); }
void __stdcall dkvrTrackerGetLed(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerLed(index); }
void __stdcall dkvrTrackerGetCalibration(DKVRHostHandle handle, int index, DKVRCalibration* out)
{
	*(reinterpret_cast<dkvr::TrackerCalibration*>(out)) = DKVRHOST(handle)->GetTrackerCalibration(index);
}
void __stdcall dkvrTrackerGetOrientation(DKVRHostHandle handle, int index, DKVRQuaternion* out)
{
	*(reinterpret_cast<dkvr::Quaternionf*>(out)) = DKVRHOST(handle)->GetTrackerOrientation(index);
}
void __stdcall dkvrTrackerGetGyro(DKVRHostHandle handle, int index, DKVRVector3* out)
{
	*(reinterpret_cast<dkvr::Vector3f*>(out)) = DKVRHOST(handle)->GetTrackerGyro(index);
}
void __stdcall dkvrTrackerGetAccel(DKVRHostHandle handle, int index, DKVRVector3* out)
{
	*(reinterpret_cast<dkvr::Vector3f*>(out)) = DKVRHOST(handle)->GetTrackerAccel(index);
}
void __stdcall dkvrTrackerGetMag(DKVRHostHandle handle, int index, DKVRVector3* out)
{
	*(reinterpret_cast<dkvr::Vector3f*>(out)) = DKVRHOST(handle)->GetTrackerMag(index);
}
void __stdcall dkvrTrackerGetExecutionTime(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerExecutionTime(index); }
void __stdcall dkvrTrackerGetInitResult(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerInitResult(index); }
void __stdcall dkvrTrackerGetBatteryPerc(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerBatteryPerc(index); }

void __stdcall dkvrTrackerSetActive(DKVRHostHandle handle, int index, int in) { DKVRHOST(handle)->SetTrackerActive(index, in); }
void __stdcall dkvrTrackerSetRaw(DKVRHostHandle handle, int index, int in) { DKVRHOST(handle)->SetTrackerRaw(index, in); }
void __stdcall dkvrTrackerSetLed(DKVRHostHandle handle, int index, int in) { DKVRHOST(handle)->SetTrackerLed(index, in); }
void __stdcall dkvrTrackerSetCalibration(DKVRHostHandle handle, int index, const DKVRCalibration* in)
{
	DKVRHOST(handle)->SetTrackerCalibration(index, *(reinterpret_cast<const dkvr::TrackerCalibration*>(in)));
}

void __stdcall dkvrTrackerRequestStatistic(DKVRHostHandle handle, int index) { DKVRHOST(handle)->RequestTrackerStatistic(index); }
void __stdcall dkvrTrackerRequestStatus(DKVRHostHandle handle, int index) { DKVRHOST(handle)->RequestTrackerStatus(index); }
void __stdcall dkvrTrackerRequestLocate(DKVRHostHandle handle, int index) { DKVRHOST(handle)->RequestTrackerLocate(index); }

// calibrator
void __stdcall dkvrCalibratorGetStatus(DKVRHostHandle handle, int* out) { *out = static_cast<int>(DKVRHOST(handle)->GetCalibratorStatus()); }
void __stdcall dkvrCalibratorGetStatusString(DKVRHostHandle handle, char* out, int len)
{
	StringCopy(DKVRHOST(handle)->GetCalibratorStatusAsString(), out, len);
}
void __stdcall dkvrCalibratorGetRequiredSampleType(DKVRHostHandle handle, int* out) { *out = static_cast<int>(DKVRHOST(handle)->GetCalibratorRequiredSampleType()); }
void __stdcall dkvrCalibratorGetRequiredSampleTypeString(DKVRHostHandle handle, char* out, int len)
{
	StringCopy(DKVRHOST(handle)->GetCalibratorRequiredSampleTypeAsString(), out, len);
}
void __stdcall dkvrCalibratorGetCurrentTarget(DKVRHostHandle handle, int* out) { *out = DKVRHOST(handle)->GetCalibratorTarget(); }
void __stdcall dkvrCalibratorBeginWith(DKVRHostHandle handle, int index) { DKVRHOST(handle)->BeginCalibrationWith(index); }
void __stdcall dkvrCalibratorAbort(DKVRHostHandle handle) { DKVRHOST(handle)->AbortCalibration(); }
void __stdcall dkvrCalibratorContinue(DKVRHostHandle handle) { DKVRHOST(handle)->ContinueCalibration(); }
