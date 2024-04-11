#pragma once

#include <atomic>
#include <memory>
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
		InstructionDispatcher(NetworkService& net_service, TrackerProvider& tk_provider);

		void Run();
		void Stop();

	private:
		void DispatcherThreadLoop();
		void Dispatch(unsigned long address, Instruction& inst);

		std::unique_ptr<std::thread> thread_ptr_;
		std::atomic_bool exit_flag_;
		InstructionHandler inst_handle_;

		NetworkService& net_service_;
		TrackerProvider& tk_provider_;
		Logger& logger_ = Logger::GetInstance();
	};

}	// namespace dkvr