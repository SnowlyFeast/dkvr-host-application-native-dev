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

    template<typename Src, typename Dst>
    void ReinterpretCast(Dst* dst, Src src)
    {
        *(reinterpret_cast<Src*>(dst)) = src;
    }
}

namespace dkvr {

    class DKVRHost
    {;
    public:
        DKVRHost();

        // instance control
        void Run();
        void Stop();
        bool IsRunning() const { return is_running_; }

        // logger
        void SetLoggerLevelDebug() { logger_.set_level(dkvr::Logger::Level::Debug); }
        void SetLoggerLevelInfo()  { logger_.set_level(dkvr::Logger::Level::Info); }
        void SetLoggerLevelError() { logger_.set_level(dkvr::Logger::Level::Error); }

        int         GetUncheckedLogCount() const { return logger_.GetUncheckedCount(); }
        std::string GetUncheckedLogOne() { logger_output_.str(std::string()); logger_output_.clear(); logger_.PrintUnchecked(1); return logger_output_.str(); }
        std::string GetUncheckedLogAll() { logger_output_.str(std::string()); logger_output_.clear(); logger_.PrintUnchecked();  return logger_output_.str(); }


        // tracker
        int	            GetTrackerCount() const             { return tk_provider_.GetCount(); }

        unsigned long   GetTrackerIPAdress(int index) const { return FindTrackerAndGet(index, &Tracker::address); }
        std::string     GetTrackerName(int index) const     { return FindTrackerAndGet(index, &Tracker::name, std::string("")); }
        Tracker::ConnectionStatus GetTrackerConnectionStatus(int index) const { return FindTrackerAndGet(index, &Tracker::connection_status); }
        long long       GetTrackerRtt(int index) const      { return FindTrackerAndGet(index, &Tracker::rtt, -1LL); }

        int GetTrackerInitResult(int index) const       { return FindTrackerAndGet(index, &Tracker::init_result); }
        int GetTrackerBatteryPerc(int index) const      { return FindTrackerAndGet(index, &Tracker::battery_perc); }

        int GetTrackerExecutionTime(int index) const    { return FindTrackerAndGet(index, &Tracker::execution_time); }

        bool GetTrackerBehaviorLed(int index) const     { return FindTrackerAndGet(index, &Tracker::behavior_led); }
        bool GetTrackerBehaviorAcitve(int index) const  { return FindTrackerAndGet(index, &Tracker::behavior_active); }
        bool GetTrackerBehaviorRaw(int index) const     { return FindTrackerAndGet(index, &Tracker::behavior_raw); }
        bool GetTrackerBehaviorNominal(int index) const { return FindTrackerAndGet(index, &Tracker::behavior_nominal); }
        void SetTrackerBehaviorLed(int index, bool led)         { FindTrackerAndSet(index, &Tracker::set_behavior_led, led); }
        void SetTrackerBehaviorActive(int index, bool active)   { FindTrackerAndSet(index, &Tracker::set_behavior_active, active); }
        void SetTrackerBehaviorRaw(int index, bool raw)         { FindTrackerAndSet(index, &Tracker::set_behavior_raw, raw); }
        void SetTrackerBehaviorNominal(int index, bool nominal) { FindTrackerAndSet(index, &Tracker::set_behavior_nominal, nominal); }
        TrackerCalibration GetTrackerCalibration(int index) const { return FindTrackerAndGet(index, &Tracker::calibration, dkvr::TrackerCalibration{}); }
        void SetTrackerCalibration(int index, const TrackerCalibration& calib) { FindTrackerAndSet(index, &Tracker::set_calibration, calib); }

        Vector3f    GetTrackerRawGyro(int index) const              { return FindTrackerAndGet(index, &Tracker::raw_gyro, Vector3f{}); }
        Vector3f    GetTrackerRawAccel(int index) const             { return FindTrackerAndGet(index, &Tracker::raw_accel, Vector3f{}); }
        Vector3f    GetTrackerRawMag(int index) const               { return FindTrackerAndGet(index, &Tracker::raw_mag, Vector3f{}); }
        Quaternionf GetTrackerOrientation(int index) const          { return FindTrackerAndGet(index, &Tracker::orientation, Quaternionf{}); }
        Vector3f    GetTrackerLinearAcceleration(int index) const   { return FindTrackerAndGet(index, &Tracker::linear_acceleration, Vector3f{}); }
        Vector3f    GetTrackerMagneticDisturbance(int index) const  { return FindTrackerAndGet(index, &Tracker::magnetic_disturbance, Vector3f{}); }

        void RequestTrackerStatistic(int index) { FindTrackerAndCall(index, &Tracker::RequestStatisticUpdate); }
        void RequestTrackerStatus(int index) { FindTrackerAndCall(index, &Tracker::RequestStatusUpdate); }
        void RequestTrackerLocate(int index) { FindTrackerAndCall(index, &Tracker::RequestLocate); }

        // calibrator
        CalibrationManager::CalibratorStatus GetCalibratorStatus() const { return calib_manager_.GetStatus(); }
        std::string GetCalibratorStatusAsString() const             { return calib_manager_.GetStatusAsString(); }
        SampleType  GetCalibratorSampleType() const                 { return calib_manager_.GetRequiredSampleType(); }
        std::string GetCalibratorSampleTypeAsString() const         { return calib_manager_.GetRequiredSampleTypeAsString(); }
        int         GetCalibratorProgress() const   { return calib_manager_.GetProgressPercentage(); }
        int         GetCalibratorTarget() const     { return calib_manager_.GetCurrentCalibrationTarget(); }
        void        BeginCalibrationWith(int index) { calib_manager_.Begin(index); }
        void        AbortCalibration()              { calib_manager_.Abort(); }
        void        ContinueCalibration()           { calib_manager_.Continue(); }

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
    catch (const std::runtime_error& except)
    {
        throw;	// just rethrow it
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

static_assert(std::is_trivial_v        <DKVRVector3>);
static_assert(std::is_standard_layout_v<DKVRVector3>);
static_assert(sizeof DKVRVector3 == sizeof dkvr::Vector3f);

static_assert(std::is_trivial_v        <DKVRQuaternion>);
static_assert(std::is_standard_layout_v<DKVRQuaternion>);
static_assert(sizeof DKVRQuaternion == sizeof dkvr::Quaternionf);

static_assert(std::is_trivial_v        <DKVRCalibration>);
static_assert(std::is_standard_layout_v<DKVRCalibration>);
static_assert(sizeof DKVRCalibration == sizeof dkvr::TrackerCalibration);
static_assert(offsetof(DKVRCalibration, gyr_transform)  == offsetof(dkvr::TrackerCalibration, gyr_transform));
static_assert(offsetof(DKVRCalibration, acc_transform)  == offsetof(dkvr::TrackerCalibration, acc_transform));
static_assert(offsetof(DKVRCalibration, mag_transform)  == offsetof(dkvr::TrackerCalibration, mag_transform));
static_assert(offsetof(DKVRCalibration, noise_variance) == offsetof(dkvr::TrackerCalibration, noise_variance));

// version
void __stdcall dkvrGetVersion(int* out)                             { *out     = DKVR_HOST_VERSION; }
void __stdcall dkvrAssertVersion(int version, int* success)         { *success = (DKVR_HOST_VERSION == version); }

// instance control
void __stdcall dkvrCreateInstance(DKVRHostHandle* hptr, char* msg, int len) {
    try { *hptr = new dkvr::DKVRHost(); }
    catch (const std::exception& except)
    {
        *hptr = nullptr;
        std::string what(except.what());
        StringCopy(what, msg, len);
    }
}
void __stdcall dkvrDeleteInstance(DKVRHostHandle* hptr)             { delete *hptr; *hptr = nullptr; }
void __stdcall dkvrRunHost(DKVRHostHandle handle)                   { DKVRHOST(handle)->Run(); }
void __stdcall dkvrStopHost(DKVRHostHandle handle)                  { DKVRHOST(handle)->Stop(); }
void __stdcall dkvrIsRunning(DKVRHostHandle handle, int* running)   { *running = (DKVRHOST(handle)->IsRunning()); }

// logger
void __stdcall dkvrLoggerSetLevelDebug(DKVRHostHandle handle)       { DKVRHOST(handle)->SetLoggerLevelDebug(); }
void __stdcall dkvrLoggerSetLevelInfo(DKVRHostHandle handle)        { DKVRHOST(handle)->SetLoggerLevelInfo(); }
void __stdcall dkvrLoggerSetLevelError(DKVRHostHandle handle)       { DKVRHOST(handle)->SetLoggerLevelError(); }

void __stdcall dkvrLoggerGetUncheckCount(DKVRHostHandle handle, int* out)                   { *out = DKVRHOST(handle)->GetUncheckedLogCount(); }
void __stdcall dkvrLoggerGetUncheckedLogOne(DKVRHostHandle handle, char* out, int len)      { StringCopy(DKVRHOST(handle)->GetUncheckedLogOne(), out, len); }
void __stdcall dkvrLoggerGetUncheckedLogAll(DKVRHostHandle handle, char* out, int len)      { StringCopy(DKVRHOST(handle)->GetUncheckedLogAll(), out, len); }

// tracker
void __stdcall dkvrTrackerGetCount(DKVRHostHandle handle, int* out)                         { *out = DKVRHOST(handle)->GetTrackerCount(); }
void __stdcall dkvrTrackerGetAddress(DKVRHostHandle handle, int index, long* out)           { *out = DKVRHOST(handle)->GetTrackerIPAdress(index); }
void __stdcall dkvrTrackerGetName(DKVRHostHandle handle, int index, char* out, int len)     { StringCopy(DKVRHOST(handle)->GetTrackerName(index), out, len); }
void __stdcall dkvrTrackerGetConnectionStatus(DKVRHostHandle handle, int index, int* out)   { *out = static_cast<int>(DKVRHOST(handle)->GetTrackerConnectionStatus(index)); }
void __stdcall dkvrTrackerGetRtt(DKVRHostHandle handle, int index, int* out)                { *out = static_cast<int>(DKVRHOST(handle)->GetTrackerRtt(index)); }

void __stdcall dkvrTrackerGetExecutionTime(DKVRHostHandle handle, int index, int* out)      { *out = DKVRHOST(handle)->GetTrackerExecutionTime(index); }
void __stdcall dkvrTrackerGetInitResult(DKVRHostHandle handle, int index, int* out)         { *out = DKVRHOST(handle)->GetTrackerInitResult(index); }
void __stdcall dkvrTrackerGetBatteryPerc(DKVRHostHandle handle, int index, int* out)        { *out = DKVRHOST(handle)->GetTrackerBatteryPerc(index); }

void __stdcall dkvrTrackerGetBehaviorLed(DKVRHostHandle handle, int index, int* out)        { *out = DKVRHOST(handle)->GetTrackerBehaviorLed(index); }
void __stdcall dkvrTrackerGetBehaviorActive(DKVRHostHandle handle, int index, int* out)     { *out = DKVRHOST(handle)->GetTrackerBehaviorAcitve(index); }
void __stdcall dkvrTrackerGetBehaviorRaw(DKVRHostHandle handle, int index, int* out)        { *out = DKVRHOST(handle)->GetTrackerBehaviorRaw(index); }
void __stdcall dkvrTrackerGetBehaviorNominal(DKVRHostHandle handle, int index, int* out)    { *out = DKVRHOST(handle)->GetTrackerBehaviorNominal(index); }
void __stdcall dkvrTrackerSetBehaviorLed(DKVRHostHandle handle, int index, int in)          { DKVRHOST(handle)->SetTrackerBehaviorLed(index, in); }
void __stdcall dkvrTrackerSetBehaviorActive(DKVRHostHandle handle, int index, int in)       { DKVRHOST(handle)->SetTrackerBehaviorActive(index, in); }
void __stdcall dkvrTrackerSetBehaviorRaw(DKVRHostHandle handle, int index, int in)          { DKVRHOST(handle)->SetTrackerBehaviorRaw(index, in); }
void __stdcall dkvrTrackerSetBehaviorNominal(DKVRHostHandle handle, int index, int in)      { DKVRHOST(handle)->SetTrackerBehaviorNominal(index, in); }

void __stdcall dkvrTrackerGetCalibration(DKVRHostHandle handle, int index, DKVRCalibration* out) { ReinterpretCast(out, DKVRHOST(handle)->GetTrackerCalibration(index)); }
void __stdcall dkvrTrackerSetCalibration(DKVRHostHandle handle, int index, const DKVRCalibration* in)
{
    DKVRHOST(handle)->SetTrackerCalibration(index, *(reinterpret_cast<const dkvr::TrackerCalibration*>(in)));
}

void __stdcall dkvrTrackerGetRawGyro(DKVRHostHandle handle, int index, DKVRVector3* out)        { ReinterpretCast(out, DKVRHOST(handle)->GetTrackerRawGyro(index)); }
void __stdcall dkvrTrackerGetRawAccel(DKVRHostHandle handle, int index, DKVRVector3* out)       { ReinterpretCast(out, DKVRHOST(handle)->GetTrackerRawAccel(index)); }
void __stdcall dkvrTrackerGetRawMag(DKVRHostHandle handle, int index, DKVRVector3* out)         { ReinterpretCast(out, DKVRHOST(handle)->GetTrackerRawMag(index)); }
void __stdcall dkvrTrackerGetOrientation(DKVRHostHandle handle, int index, DKVRQuaternion* out) { ReinterpretCast(out, DKVRHOST(handle)->GetTrackerOrientation(index)); }

void __stdcall dkvrTrackerRequestLocate(DKVRHostHandle handle, int index)       { DKVRHOST(handle)->RequestTrackerLocate(index); }
void __stdcall dkvrTrackerRequestStatus(DKVRHostHandle handle, int index)       { DKVRHOST(handle)->RequestTrackerStatus(index); }
void __stdcall dkvrTrackerRequestStatistic(DKVRHostHandle handle, int index)    { DKVRHOST(handle)->RequestTrackerStatistic(index); }

// calibrator
void __stdcall dkvrCalibratorGetStatus(DKVRHostHandle handle, int* out)                     { *out = static_cast<int>(DKVRHOST(handle)->GetCalibratorStatus()); }
void __stdcall dkvrCalibratorGetSampleType(DKVRHostHandle handle, int* out)                 { *out = static_cast<int>(DKVRHOST(handle)->GetCalibratorSampleType()); }
void __stdcall dkvrCalibratorGetStatusString(DKVRHostHandle handle, char* out, int len)     { StringCopy(DKVRHOST(handle)->GetCalibratorStatusAsString(), out, len); }
void __stdcall dkvrCalibratorGetSampleTypeString(DKVRHostHandle handle, char* out, int len) { StringCopy(DKVRHOST(handle)->GetCalibratorSampleTypeAsString(), out, len); }
void __stdcall dkvrCalibratorGetProgress(DKVRHostHandle handle, int* out)                   { *out = DKVRHOST(handle)->GetCalibratorProgress(); }
void __stdcall dkvrCalibratorGetCurrentTarget(DKVRHostHandle handle, int* out)              { *out = DKVRHOST(handle)->GetCalibratorTarget(); }
void __stdcall dkvrCalibratorBeginWith(DKVRHostHandle handle, int index)                    { DKVRHOST(handle)->BeginCalibrationWith(index); }
void __stdcall dkvrCalibratorAbort(DKVRHostHandle handle)                                   { DKVRHOST(handle)->AbortCalibration(); }
void __stdcall dkvrCalibratorContinue(DKVRHostHandle handle)                                { DKVRHOST(handle)->ContinueCalibration(); }
