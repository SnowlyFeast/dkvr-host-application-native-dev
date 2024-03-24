#pragma once

#include <atomic>
#include <thread>

#include "con_config.h"
#include "con_data.h"
#include "con_miscell.h"
#include "con_network.h"
#include "instruction_format.h"
#include "network/network_service.h"
#include "tracker/tracker_provider.h"
#include "util/logger.h"

namespace dkvr {

	class InstructionDispatcher
	{
	public:
		InstructionDispatcher() : thread_(nullptr), exit_flag_(false), network_con_(), miscel_con_(), config_con_(), data_con_() { }

		void Run();
		void Stop();

	private:
		void DispatcherThreadLoop();
		void Dispatch(unsigned long address, Instruction& inst);

		std::thread* thread_;
		std::atomic_bool exit_flag_;
		NetworkingController network_con_;
		MiscellanousController miscel_con_;
		ConfigurationController config_con_;
		DataTransferController data_con_;

		NetworkService& net_service_ = NetworkService::GetInstance();
		TrackerProvider& tk_provider_ = TrackerProvider::GetInstance();
		Logger& logger_ = Logger::GetInstance();
	};

}	// namespace dkvr