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

	// version
	DLLEXPORT void dkvrVersion(int* out);
	DLLEXPORT void dkvrAssertVersion(int version, int* success);

	// instance control
	DLLEXPORT void dkvrCreateInstance(DKVRHostHandle* hptr);
	DLLEXPORT void dkvrDeleteInstance(DKVRHostHandle* hptr);
	DLLEXPORT void dkvrRunHost(DKVRHostHandle handle);
	DLLEXPORT void dkvrStopHost(DKVRHostHandle handle);
	DLLEXPORT void dkvrIsRunning(DKVRHostHandle handle, int* running);

	// logger
	DLLEXPORT void dkvrLoggerGetUncheckCount(DKVRHostHandle handle, int* out);
	DLLEXPORT void dkvrLoggerGetUncheckedLogOne(DKVRHostHandle handle, char* out, int len);
	DLLEXPORT void dkvrLoggerGetUncheckedLogAll(DKVRHostHandle handle, char* out, int len);

	DLLEXPORT void dkvrLoggerSetLevelDebug(DKVRHostHandle handle);
	DLLEXPORT void dkvrLoggerSetLevelInfo(DKVRHostHandle handle);
	DLLEXPORT void dkvrLoggerSetLevelError(DKVRHostHandle handle);

	// tracker
	DLLEXPORT void dkvrTrackerGetCount(DKVRHostHandle handle, int* out);
	DLLEXPORT void dkvrTrackerGetAddress(DKVRHostHandle handle, int index, long* out);
	DLLEXPORT void dkvrTrackerGetName(DKVRHostHandle handle, int index, char* out, int len);
	DLLEXPORT void dkvrTrackerGetConnectionStatus(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void dkvrTrackerGetRtt(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void dkvrTrackerGetAcitve(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void dkvrTrackerGetRaw(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void dkvrTrackerGetLed(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void dkvrTrackerGetQuat(DKVRHostHandle handle, int index, float* out);
	DLLEXPORT void dkvrTrackerGetGyro(DKVRHostHandle handle, int index, float* out);
	DLLEXPORT void dkvrTrackerGetAccel(DKVRHostHandle handle, int index, float* out);
	DLLEXPORT void dkvrTrackerGetMag(DKVRHostHandle handle, int index, float* out);
	DLLEXPORT void dkvrTrackerGetInitResult(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void dkvrTrackerGetLastError(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void dkvrTrackerGetBatteryPerc(DKVRHostHandle handle, int index, int* out);

	DLLEXPORT void dkvrTrackerSetActive(DKVRHostHandle handle, int index, int in);
	DLLEXPORT void dkvrTrackerSetRaw(DKVRHostHandle handle, int index, int in);
	DLLEXPORT void dkvrTrackerSetLed(DKVRHostHandle handle, int index, int in);

	// calibrator
	DLLEXPORT void dkvrCalibratorGetStatus(DKVRHostHandle handle, int* out);
	DLLEXPORT void dkvrCalibratorGetCurrentTarget(DKVRHostHandle handle, int* out);
	DLLEXPORT void dkvrCalibratorBeginWith(DKVRHostHandle handle, int index);
	DLLEXPORT void dkvrCalibratorAbort(DKVRHostHandle handle);

#ifdef __cplusplus
}
#endif