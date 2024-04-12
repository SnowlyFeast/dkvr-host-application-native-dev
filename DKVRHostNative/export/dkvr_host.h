#pragma once

#ifdef DKVRHOSTNATIVE_EXPORTS
#	define DLLEXPORT	__declspec( dllexport )
#else
#	define DLLEXPORT	__declspec( dllimport )
#endif

#define DKVR_HOST_HEADER_VER	1001

#ifdef __cplusplus
extern "C" {
#endif

	typedef void* DKVRHostHandle;

	DLLEXPORT int dkvrVersion();
	DLLEXPORT int dkvrAssertVersion(int version);

	DLLEXPORT void dkvrCreateInstance(DKVRHostHandle handle);
	DLLEXPORT void dkvrDeleteInstance(DKVRHostHandle handle);
	DLLEXPORT void dkvrRunHost(DKVRHostHandle handle);
	DLLEXPORT void dkvrIsRunning(DKVRHostHandle handle, int* result);
	DLLEXPORT void dkvrStopHost(DKVRHostHandle handle);

	DLLEXPORT void dkvrTrackerCount(DKVRHostHandle handle, int* out);
	DLLEXPORT void dkvrTrackerAddress(DKVRHostHandle handle, int index, long* out);
	DLLEXPORT void dkvrTrackerName(DKVRHostHandle handle, int index, char* out, int len);
	DLLEXPORT void dkvrTrackerConnectionStatus(DKVRHostHandle handle, int index, int* out);

	DLLEXPORT void dkvrTrackerRtt(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void dkvrTrackerAcitve(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void dkvrTrackerRaw(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void dkvrTrackerLed(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void dkvrTrackerQuat(DKVRHostHandle handle, int index, float* out);
	DLLEXPORT void dkvrTrackerGyro(DKVRHostHandle handle, int index, float* out);
	DLLEXPORT void dkvrTrackerAccel(DKVRHostHandle handle, int index, float* out);
	DLLEXPORT void dkvrTrackerMag(DKVRHostHandle handle, int index, float* out);
	DLLEXPORT void dkvrTrackerInitResult(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void dkvrTrackerLastError(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void dkvrTrackerBatteryPerc(DKVRHostHandle handle, int index, int* out);

	DLLEXPORT void dkvrTrackerSetActive(DKVRHostHandle handle, int index, int in);
	DLLEXPORT void dkvrTrackerSetRaw(DKVRHostHandle handle, int index, int in);
	DLLEXPORT void dkvrTrackerSetLed(DKVRHostHandle handle, int index, int in);

#ifdef __cplusplus
}
#endif

