#pragma once

#include <atomic>
#include <thread>

#include "controller/instruction_format.h"
#include "controller/instruction_handler.h"
#include "network/network_service.h"
#include "tracker/tracker_provider.h"
#include "util/logger.h"

namespace dkvr {

	class InstructionDispatcher
	{
	public:
		InstructionDispatcher() : thread_(nullptr), exit_flag_(false), inst_handle_() { }

		void Run();
		void Stop();

	private:
		void DispatcherThreadLoop();
		void Dispatch(unsigned long address, Instruction& inst);

		std::thread* thread_;
		std::atomic_bool exit_flag_;
		InstructionHandler inst_handle_;

		NetworkService& net_service_ = NetworkService::GetInstance();
		TrackerProvider& tk_provider_ = TrackerProvider::GetInstance();
		Logger& logger_ = Logger::GetInstance();
	};

}	// namespace dkvr