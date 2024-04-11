#pragma once

#include <stdexcept>

#include "calibrator/calibration_manager.h"
#include "controller/instruction_dispatcher.h"
#include "controller/tracker_updater.h"
#include "network/network_service.h"
#include "tracker/tracker_provider.h"
#include "util/logger.h"

namespace dkvr
{

	class DKVRHost
	{
	public:
		DKVRHost();

		void Run();
		void Stop();

		bool IsRunning() const;

		

	private:
		NetworkService net_service_;
		TrackerProvider tk_provider_;
		InstructionDispatcher inst_dispatcher_;
		TrackerUpdater tracker_updater_;
		CalibrationManager calib_manager_;
		Logger& logger_ = Logger::GetInstance();

		bool is_running_ = false;
	};

}	// namespace dkvr