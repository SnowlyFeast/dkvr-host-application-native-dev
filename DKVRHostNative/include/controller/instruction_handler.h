#pragma once

#include <string>

#include "controller/instruction_format.h"
#include "tracker/tracker.h"
#include "util/logger.h"

namespace dkvr {

	class InstructionHandler
	{
	public:
		void Handle(Tracker* target, Instruction& inst);

	private:
		void Handshake1(Tracker* target, Instruction& inst);
		void Handshake2(Tracker* target, Instruction& inst);
		void Heartbeat(Tracker* target, Instruction& inst);
		void Ping(Tracker* target, Instruction& inst);
		void Pong(Tracker* target, Instruction& inst);

		void Locate(Tracker* target, Instruction& inst);

		void Behavior(Tracker* target, Instruction& inst);
		void CalibrationGr(Tracker* target, Instruction& inst);
		void CalibrationAc(Tracker* target, Instruction& inst);
		void CalibrationMg(Tracker* target, Instruction& inst);

		void Status(Tracker* target, Instruction& inst);
		void ImuRaw(Tracker* target, Instruction& inst);
		void ImuQuat(Tracker* target, Instruction& inst);

		Logger& logger_ = Logger::GetInstance();
	};

}	// namespace dkvr