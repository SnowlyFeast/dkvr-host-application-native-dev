#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <sstream>
#include <string>

#ifdef _DEBUG
#	define DKVR_DEBUG_TRACKER_CONNECTION_DETAIL
#else
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
		CalibrationMatrix GetTrackerCalibration(int index) const { return FindTrackerAndGet(index, &Tracker::calibration, dkvr::CalibrationMatrix{}); }
		Quaternion GetTrackerQuat(int index) const { return FindTrackerAndGet(index, &Tracker::quaternion, Quaternion{}); }
		Vector3 GetTrackerGyro(int index) const { return FindTrackerAndGet(index, &Tracker::gyro, Vector3{}); }
		Vector3 GetTrackerAccel(int index) const { return FindTrackerAndGet(index, &Tracker::accel, Vector3{}); }
		Vector3 GetTrackerMag(int index) const { return FindTrackerAndGet(index, &Tracker::mag, Vector3{}); }
		int GetTrackerExecutionTime(int index) const { return FindTrackerAndGet(index, &Tracker::execution_time); }
		int GetTrackerInitResult(int index) const { return FindTrackerAndGet(index, &Tracker::init_result); }
		int GetTrackerLastError(int index) const { return FindTrackerAndGet(index, &Tracker::last_err); }
		int GetTrackerBatteryPerc(int index) const { return FindTrackerAndGet(index, &Tracker::battery_perc); }

		void SetTrackerActive(int index, bool active) { FindTrackerAndSet(index, &Tracker::set_active, active); }
		void SetTrackerRaw(int index, bool raw) { FindTrackerAndSet(index, &Tracker::set_raw, raw); }
		void SetTrackerLed(int index, bool led) { FindTrackerAndSet(index, &Tracker::set_led, led); }
		void SetTrackerCalibration(int index, const CalibrationMatrix& calib) { FindTrackerAndSet(index, &Tracker::set_calibration, calib); }

		void RequestTrackerStatistic(int index) { FindTrackerAndSet(index, &Tracker::RequestStatisticUpdate); }
		void RequestTrackerStatus(int index) { FindTrackerAndSet(index, &Tracker::RequestStatusUpdate); }
		void RequestTrackerLocate(int index) { FindTrackerAndSet(index, &Tracker::RequestLocate); }
		void RequestTrackerMagRefRecalc(int index) { FindTrackerAndSet(index, &Tracker::RequestMagRefRecalc); }

		// calibrator
		CalibrationManager::CalibrationStatus GetCalibratorStatus() { return calib_manager_.GetStatus(); }
		std::string GetCalibratorStatusAsString() { return calib_manager_.GetStatusAsString(); }
		CalibrationManager::SampleTypes GetCalibratorRequiredSampleType() { return calib_manager_.GetRequiredSampleType(); }
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

		void FindTrackerAndSet(int index, void(Tracker::* setter)(void))
		{
			AtomicTracker target = tk_provider_.FindByIndex(index);
			if (target)
				(target->*setter)();
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

// export struct matching assertion
static_assert(sizeof Calibration == sizeof dkvr::CalibrationMatrix);
static_assert(sizeof Calibration::gyro == sizeof dkvr::CalibrationMatrix::gyr);
static_assert(sizeof Calibration::accel == sizeof dkvr::CalibrationMatrix::acc);
static_assert(sizeof Calibration::mag == sizeof dkvr::CalibrationMatrix::mag);
static_assert(sizeof Quaternion == sizeof dkvr::Quaternion);
static_assert(sizeof Vector3 == sizeof dkvr::Vector3);

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
	std::string log = DKVRHOST(handle)->GetUncheckedLogOne();
	size_t cap = std::min(log.size() + 1, static_cast<size_t>(len));
	memcpy_s(out, len, log.c_str(), cap);
	out[cap - 1] = '\0';
}
void __stdcall dkvrLoggerGetUncheckedLogAll(DKVRHostHandle handle, char* out, int len)
{
	std::string log = DKVRHOST(handle)->GetUncheckedLogAll();
	size_t cap = std::min(log.size() + 1, static_cast<size_t>(len));
	memcpy_s(out, len, log.c_str(), cap);
	out[cap - 1] = '\0';
}

// tracker
void __stdcall dkvrTrackerGetCount(DKVRHostHandle handle, int* out) { *out = DKVRHOST(handle)->GetTrackerCount(); }
void __stdcall dkvrTrackerGetAddress(DKVRHostHandle handle, int index, long* out) { *out = DKVRHOST(handle)->GetTrackerIPAdress(index); }
void __stdcall dkvrTrackerGetName(DKVRHostHandle handle, int index, char* out, int len)
{
	std::string name = DKVRHOST(handle)->GetTrackerName(index);
	size_t cap = std::min(name.size() + 1, static_cast<size_t>(len));
	memcpy_s(out, len, name.c_str(), cap);
	out[cap - 1] = '\0';
}
void __stdcall dkvrTrackerGetConnectionStatus(DKVRHostHandle handle, int index, int* out) { *out = static_cast<int>(DKVRHOST(handle)->GetTrackerConnectionStatus(index)); }
void __stdcall dkvrTrackerGetRtt(DKVRHostHandle handle, int index, int* out) { *out = static_cast<int>(DKVRHOST(handle)->GetTrackerRtt(index)); }
void __stdcall dkvrTrackerGetActive(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerAcitve(index); }
void __stdcall dkvrTrackerGetRaw(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerRaw(index); }
void __stdcall dkvrTrackerGetLed(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerLed(index); }
void __stdcall dkvrTrackerGetCalibration(DKVRHostHandle handle, int index, Calibration* out)
{
	dkvr::CalibrationMatrix calib = DKVRHOST(handle)->GetTrackerCalibration(index);
	memcpy_s(out, sizeof Calibration, &calib, sizeof dkvr::CalibrationMatrix);
}
void __stdcall dkvrTrackerGetQuat(DKVRHostHandle handle, int index, Quaternion* out)
{
	dkvr::Quaternion quat = DKVRHOST(handle)->GetTrackerQuat(index);
	memcpy_s(out, sizeof Quaternion, &quat, sizeof dkvr::Quaternion);
}
void __stdcall dkvrTrackerGetGyro(DKVRHostHandle handle, int index, Vector3* out)
{
	dkvr::Vector3 vec = DKVRHOST(handle)->GetTrackerGyro(index);
	memcpy_s(out, sizeof Vector3, &vec, sizeof dkvr::Vector3);
}
void __stdcall dkvrTrackerGetAccel(DKVRHostHandle handle, int index, Vector3* out)
{
	dkvr::Vector3 vec = DKVRHOST(handle)->GetTrackerAccel(index);
	memcpy_s(out, sizeof Vector3, &vec, sizeof dkvr::Vector3);
}
void __stdcall dkvrTrackerGetMag(DKVRHostHandle handle, int index, Vector3* out)
{
	dkvr::Vector3 vec = DKVRHOST(handle)->GetTrackerMag(index);
	memcpy_s(out, sizeof Vector3, &vec, sizeof dkvr::Vector3);
}
void __stdcall dkvrTrackerGetExecutionTime(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerExecutionTime(index); }
void __stdcall dkvrTrackerGetInitResult(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerInitResult(index); }
void __stdcall dkvrTrackerGetLastError(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerLastError(index); }
void __stdcall dkvrTrackerGetBatteryPerc(DKVRHostHandle handle, int index, int* out) { *out = DKVRHOST(handle)->GetTrackerBatteryPerc(index); }

void __stdcall dkvrTrackerSetActive(DKVRHostHandle handle, int index, int in) { DKVRHOST(handle)->SetTrackerActive(index, in); }
void __stdcall dkvrTrackerSetRaw(DKVRHostHandle handle, int index, int in) { DKVRHOST(handle)->SetTrackerRaw(index, in); }
void __stdcall dkvrTrackerSetLed(DKVRHostHandle handle, int index, int in) { DKVRHOST(handle)->SetTrackerLed(index, in); }
void __stdcall dkvrTrackerSetCalibration(DKVRHostHandle handle, int index, const Calibration* in)
{
	DKVRHOST(handle)->SetTrackerCalibration(index, *(reinterpret_cast<const dkvr::CalibrationMatrix*>(in)));
}

void __stdcall dkvrTrackerRequestStatistic(DKVRHostHandle handle, int index) { DKVRHOST(handle)->RequestTrackerStatistic(index); }
void __stdcall dkvrTrackerRequestStatus(DKVRHostHandle handle, int index) { DKVRHOST(handle)->RequestTrackerStatus(index); }
void __stdcall dkvrTrackerRequestLocate(DKVRHostHandle handle, int index) { DKVRHOST(handle)->RequestTrackerLocate(index); }
void __stdcall dkvrTrackerRequestMagRefRecalc(DKVRHostHandle handle, int index) { DKVRHOST(handle)->RequestTrackerMagRefRecalc(index); }

// calibrator
void __stdcall dkvrCalibratorGetStatus(DKVRHostHandle handle, int* out) { *out = static_cast<int>(DKVRHOST(handle)->GetCalibratorStatus()); }
void __stdcall dkvrCalibratorGetStatusString(DKVRHostHandle handle, char* out, int len)
{
	std::string status = DKVRHOST(handle)->GetCalibratorStatusAsString();
	size_t cap = std::min(status.size() + 1, static_cast<size_t>(len));
	memcpy_s(out, len, status.c_str(), cap);
	out[cap - 1] = '\0';
}
void __stdcall dkvrCalibratorGetRequiredSampleType(DKVRHostHandle handle, int* out) { *out = static_cast<int>(DKVRHOST(handle)->GetCalibratorRequiredSampleType()); }
void __stdcall dkvrCalibratorGetRequiredSampleTypeString(DKVRHostHandle handle, char* out, int len)
{
	std::string type = DKVRHOST(handle)->GetCalibratorRequiredSampleTypeAsString();
	size_t cap = std::min(type.size() + 1, static_cast<size_t>(len));
	memcpy_s(out, len, type.c_str(), cap);
	out[cap - 1] = '\0';
}
void __stdcall dkvrCalibratorGetCurrentTarget(DKVRHostHandle handle, int* out) { *out = DKVRHOST(handle)->GetCalibratorTarget(); }
void __stdcall dkvrCalibratorBeginWith(DKVRHostHandle handle, int index) { DKVRHOST(handle)->BeginCalibrationWith(index); }
void __stdcall dkvrCalibratorAbort(DKVRHostHandle handle) { DKVRHOST(handle)->AbortCalibration(); }
void __stdcall dkvrCalibratorContinue(DKVRHostHandle handle) { DKVRHOST(handle)->ContinueCalibration(); }
