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
	typedef struct Vector3_s { float x, y, z; } Vector3;
	typedef struct Quaternion_s { float x, y, z, w; } Quaternion;
	typedef struct Calibration_s { float gyro[3], accel[12], mag[12]; } Calibration;

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
	DLLEXPORT void __stdcall dkvrTrackerGetCalibration(DKVRHostHandle handle, int index, Calibration* out);
	DLLEXPORT void __stdcall dkvrTrackerGetQuat(DKVRHostHandle handle, int index, Quaternion* out);
	DLLEXPORT void __stdcall dkvrTrackerGetGyro(DKVRHostHandle handle, int index, Vector3* out);
	DLLEXPORT void __stdcall dkvrTrackerGetAccel(DKVRHostHandle handle, int index, Vector3* out);
	DLLEXPORT void __stdcall dkvrTrackerGetMag(DKVRHostHandle handle, int index, Vector3* out);
	DLLEXPORT void __stdcall dkvrTrackerGetInitResult(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void __stdcall dkvrTrackerGetLastError(DKVRHostHandle handle, int index, int* out);
	DLLEXPORT void __stdcall dkvrTrackerGetBatteryPerc(DKVRHostHandle handle, int index, int* out);

	DLLEXPORT void __stdcall dkvrTrackerSetActive(DKVRHostHandle handle, int index, int in);
	DLLEXPORT void __stdcall dkvrTrackerSetRaw(DKVRHostHandle handle, int index, int in);
	DLLEXPORT void __stdcall dkvrTrackerSetLed(DKVRHostHandle handle, int index, int in);
	DLLEXPORT void __stdcall dkvrTrackerSetCalibration(DKVRHostHandle handle, int index, const Calibration* in);

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