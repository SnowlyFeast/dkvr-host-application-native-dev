#pragma once

#ifdef DKVRHOSTNATIVE_EXPORTS
#	define DLLEXPORT	__declspec( dllexport )
#else
#	define DLLEXPORT	__declspec( dllimport )
#endif

#define DKVR_HOST_EXPORTED_HEADER_VER	1001

#ifdef __cplusplus
extern "C" {
#endif

	typedef void* DKVRHostHandle;
	struct DKVRVector3 { float x, y, z; };
	struct DKVRQuaternion { float w, x, y, z; };
	struct DKVRCalibration { float gyr_transform[12], acc_transform[12], mag_transform[12], gyr_noise_var[3], acc_noise_var[3], mag_noise_var[3]; };

	// version
	DLLEXPORT void __stdcall dkvrVersion(int* out);
	DLLEXPORT void __stdcall dkvrAssertVersion(int version, int* success);

	// instance control
	DLLEXPORT void __stdcall dkvrCreateInstance(DKVRHostHandle* hptr);
	DLLEXPORT void __stdcall dkvrDeleteInstance(DKVRHostHandle* hptr);
	DLLEXPORT void __stdcall dkvrRunHost(DKVRHostHandle handle);
	DLLEXPORT void __stdcall dkvrStopHost(DKVRHostHandle handle);
	DLLEXPORT void __stdcall dkvrIsRunning(DKVRHostHandle handle, int* running);

	// logger
	DLLEXPORT void __stdcall dkvrLoggerGetUncheckCount(DKVRHostHandle handle, int* out);
	DLLEXPORT void __stdcall dkvrLoggerGetUncheckedLogOne(DKVRHostHandle handle, char* out, int len);
	DLLEXPORT void __stdcall dkvrLoggerGetUncheckedLogAll(DKVRHostHandle handle, char* out, int len);

	DLLEXPORT void __stdcall dkvrLoggerSetLevelDebug(DKVRHostHandle handle);
	DLLEXPORT void __stdcall dkvrLoggerSetLevelInfo(DKVRHostHandle handle);
	DLLEXPORT void __stdcall dkvrLoggerSetLevelError(DKVRHostHandle handle);

	// tracker
	DLLEXPORT void __stdcall dkvrTrackerGetCount(DKVRHostHandle handle, int* out);
	DLLEXPORT void __stdcall dkvrTrackerGetAddress(DKVRHostHandle handle, int index, long* out);
	DLLEXPORT void __stdcall dkvrTrackerGetName(DKVRHostHandle handle, int index, char* out, int len);
	DLLEXPORT void __stdcall dkvrTrackerGetConnectionStatus(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void __stdcall dkvrTrackerGetRtt(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void __stdcall dkvrTrackerGetActive(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void __stdcall dkvrTrackerGetRaw(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void __stdcall dkvrTrackerGetLed(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void __stdcall dkvrTrackerGetCalibration(DKVRHostHandle handle, int index, struct DKVRCalibration* out);
	DLLEXPORT void __stdcall dkvrTrackerGetOrientation(DKVRHostHandle handle, int index, struct DKVRQuaternion* out);
	DLLEXPORT void __stdcall dkvrTrackerGetGyro(DKVRHostHandle handle, int index, struct DKVRVector3* out);
	DLLEXPORT void __stdcall dkvrTrackerGetAccel(DKVRHostHandle handle, int index, struct DKVRVector3* out);
	DLLEXPORT void __stdcall dkvrTrackerGetMag(DKVRHostHandle handle, int index, struct DKVRVector3* out);
	DLLEXPORT void __stdcall dkvrTrackerGetExecutionTime(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void __stdcall dkvrTrackerGetInitResult(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void __stdcall dkvrTrackerGetBatteryPerc(DKVRHostHandle handle, int index, int* out);

	DLLEXPORT void __stdcall dkvrTrackerSetActive(DKVRHostHandle handle, int index, int in);
	DLLEXPORT void __stdcall dkvrTrackerSetRaw(DKVRHostHandle handle, int index, int in);
	DLLEXPORT void __stdcall dkvrTrackerSetLed(DKVRHostHandle handle, int index, int in);
	DLLEXPORT void __stdcall dkvrTrackerSetCalibration(DKVRHostHandle handle, int index, const struct DKVRCalibration* in);

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