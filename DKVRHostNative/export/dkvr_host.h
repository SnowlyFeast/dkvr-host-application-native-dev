#pragma once

#ifdef DKVRHOSTNATIVE_EXPORTS
#	define DLLEXPORT	__declspec( dllexport )
#else
#	define DLLEXPORT	__declspec( dllimport )
#endif

#define DKVR_HOST_EXPORTED_HEADER_VER	1003

#ifdef __cplusplus
extern "C" {
#endif

    // TODO: maybe should dynamically generate those type defs, but it's simple enough so not now
    enum DKVRLoggerMode
    {
        Echo,
        Burst,
        Silent
    };
    enum DKVRConnectionStatus
    {
        Disconnected,
        Handshaked,
        Connected
    };
    enum DKVRCalibratorStatus
    {
        Idle,
        Configuring,
        StandBy,
        Recording,
        Calibrating
    };

    struct DKVRAddress { unsigned char ip[4]; unsigned short port; };
    struct DKVRVector3 { float x, y, z; };
    struct DKVRQuaternion { float w, x, y, z; };
    struct DKVRCalibration { float gyr_transform[12], acc_transform[12], mag_transform[12], noise_variance[9]; };

	// version
	DLLEXPORT void __stdcall dkvrVersion(int* out);
	DLLEXPORT void __stdcall dkvrAssertVersion(int version, int* success);

    // instance control
    DLLEXPORT void __stdcall dkvrCreateInstance	(DKVRHostHandle* hptr, char* msg, int len);
    DLLEXPORT void __stdcall dkvrDeleteInstance	(DKVRHostHandle* hptr);
    DLLEXPORT void __stdcall dkvrRunHost		(DKVRHostHandle handle, DKVRAddress address);
    DLLEXPORT void __stdcall dkvrStopHost		(DKVRHostHandle handle);
    DLLEXPORT void __stdcall dkvrIsRunning		(DKVRHostHandle handle, int* running);

    // logger
#ifdef __cplusplus
    DLLEXPORT void __stdcall dkvrLoggerSetLoggerOutput      (DKVRHostHandle handle, std::ostream& ostream);
#endif
    DLLEXPORT void __stdcall dkvrLoggerSetLoggerMode        (DKVRHostHandle handle, DKVRLoggerMode mode);

    DLLEXPORT void __stdcall dkvrLoggerGetUncheckCount		(DKVRHostHandle handle, int* out);
    DLLEXPORT void __stdcall dkvrLoggerGetUncheckedLogOne	(DKVRHostHandle handle, char* out, int len);
    DLLEXPORT void __stdcall dkvrLoggerGetUncheckedLogAll	(DKVRHostHandle handle, char* out, int len);

	DLLEXPORT void __stdcall dkvrLoggerSetLevelDebug(DKVRHostHandle handle);
	DLLEXPORT void __stdcall dkvrLoggerSetLevelInfo(DKVRHostHandle handle);
	DLLEXPORT void __stdcall dkvrLoggerSetLevelError(DKVRHostHandle handle);

    // tracker
    DLLEXPORT void __stdcall dkvrTrackerGetCount			(DKVRHostHandle handle, int* out);
    DLLEXPORT void __stdcall dkvrTrackerGetAddress			(DKVRHostHandle handle, int index, unsigned long* out);
    DLLEXPORT void __stdcall dkvrTrackerGetName				(DKVRHostHandle handle, int index, char* out, int len);
    DLLEXPORT void __stdcall dkvrTrackerGetConnectionStatus	(DKVRHostHandle handle, int index, int* out);
    DLLEXPORT void __stdcall dkvrTrackerGetRtt				(DKVRHostHandle handle, int index, int* out);

    DLLEXPORT void __stdcall dkvrTrackerGetInitResult       (DKVRHostHandle handle, int index, int* out);
    DLLEXPORT void __stdcall dkvrTrackerGetBatteryPerc      (DKVRHostHandle handle, int index, int* out);
    DLLEXPORT void __stdcall dkvrTrackerGetExecutionTime    (DKVRHostHandle handle, int index, int* out);
    DLLEXPORT void __stdcall dkvrTrackerGetInterruptMissRate(DKVRHostHandle handle, int index, int* out);
    DLLEXPORT void __stdcall dkvrTrackerGetImuMissRate      (DKVRHostHandle handle, int index, int* out);

    DLLEXPORT void __stdcall dkvrTrackerGetBehaviorLed		(DKVRHostHandle handle, int index, int* out);
    DLLEXPORT void __stdcall dkvrTrackerGetBehaviorActive	(DKVRHostHandle handle, int index, int* out);
    DLLEXPORT void __stdcall dkvrTrackerGetBehaviorRaw		(DKVRHostHandle handle, int index, int* out);
    DLLEXPORT void __stdcall dkvrTrackerGetBehaviorNominal	(DKVRHostHandle handle, int index, int* out);
    DLLEXPORT void __stdcall dkvrTrackerSetBehaviorLed		(DKVRHostHandle handle, int index, int in);
    DLLEXPORT void __stdcall dkvrTrackerSetBehaviorActive	(DKVRHostHandle handle, int index, int in);
    DLLEXPORT void __stdcall dkvrTrackerSetBehaviorRaw		(DKVRHostHandle handle, int index, int in);
    DLLEXPORT void __stdcall dkvrTrackerSetBehaviorNominal	(DKVRHostHandle handle, int index, int in);

    DLLEXPORT void __stdcall dkvrTrackerGetCalibration		(DKVRHostHandle handle, int index, struct DKVRCalibration* out);
    DLLEXPORT void __stdcall dkvrTrackerSetCalibration		(DKVRHostHandle handle, int index, const struct DKVRCalibration* in);

    DLLEXPORT void __stdcall dkvrTrackerGetRawGyro			(DKVRHostHandle handle, int index, struct DKVRVector3* out);
    DLLEXPORT void __stdcall dkvrTrackerGetRawAccel			(DKVRHostHandle handle, int index, struct DKVRVector3* out);
    DLLEXPORT void __stdcall dkvrTrackerGetRawMag			(DKVRHostHandle handle, int index, struct DKVRVector3* out);
    DLLEXPORT void __stdcall dkvrTrackerGetOrientation		(DKVRHostHandle handle, int index, struct DKVRQuaternion* out);
    DLLEXPORT void __stdcall dkvrTrackerGetLinearAcceleration(DKVRHostHandle handle, int index, struct DKVRVector3* out);
    DLLEXPORT void __stdcall dkvrTrackerGetMagneticDisturbance(DKVRHostHandle handle, int index, struct DKVRVector3* out);

	DLLEXPORT void __stdcall dkvrTrackerRequestStatistic(DKVRHostHandle handle, int index);
	DLLEXPORT void __stdcall dkvrTrackerRequestStatus(DKVRHostHandle handle, int index);
	DLLEXPORT void __stdcall dkvrTrackerRequestLocate(DKVRHostHandle handle, int index);

	// calibrator
	DLLEXPORT void __stdcall dkvrCalibratorGetStatus(DKVRHostHandle handle, int* out);
	DLLEXPORT void __stdcall dkvrCalibratorGetStatusString(DKVRHostHandle handle, char* out, int len);
	DLLEXPORT void __stdcall dkvrCalibratorGetRequiredSampleType(DKVRHostHandle handle, int* out);
	DLLEXPORT void __stdcall dkvrCalibratorGetRequiredSampleTypeString(DKVRHostHandle handle, char* out, int len);
	DLLEXPORT void __stdcall dkvrCalibratorGetCurrentTarget(DKVRHostHandle handle, int* out);
	DLLEXPORT void __stdcall dkvrCalibratorBeginWith(DKVRHostHandle handle, int index);
	DLLEXPORT void __stdcall dkvrCalibratorAbort(DKVRHostHandle handle);
	DLLEXPORT void __stdcall dkvrCalibratorContinue(DKVRHostHandle handle);

#ifdef __cplusplus
}
#endif