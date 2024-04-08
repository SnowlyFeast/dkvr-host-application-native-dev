#pragma once

#include <cstdint>

namespace dkvr {

	constexpr uint8_t kHeaderValue = 'D';

	enum class Opcode : uint8_t
	{
		// networking op
		Handshake1		= 0x01,
		Handshake2		= 0x02,
		Heartbeat		= 0x03,
		Ping			= 0x04,
		Pong			= 0x05,

		// miscellaneous op
		Locate			= 0x11,

		// configuration op
		Active			= 0x21,
		Inactive		= 0x22,
		Behavior		= 0x23,
		CalibrationGr	= 0x24,
		CalibrationAc	= 0x25,
		CalibrationMg	= 0x26,

		// data transfer op
		Status			= 0x31,
		ImuRaw			= 0x32,
		ImuQuat			= 0x33
	};

	struct InstructionHint
	{
		Opcode opcode;
		uint8_t align;
		uint8_t arg_count;
		uint8_t length;	// length should be (align * arg_count)
	};

	class InstructionSet
	{
	public:
		static constexpr uint8_t OpcodeClassMask = 0xF0;
		static constexpr uint8_t NetworkingOp = 0x00;
		static constexpr uint8_t MiscellaneousOp = 0x10;
		static constexpr uint8_t ConfigurationOp = 0x20;
		static constexpr uint8_t DataTransferOp = 0x30;

		static constexpr InstructionHint Handshake1{ Opcode::Handshake1, 0, 0, 0 };
		static constexpr InstructionHint Handshake2{ Opcode::Handshake2, 0, 0, 0 };
		static constexpr InstructionHint Heartbeat{ Opcode::Heartbeat, 0, 0, 0 };
		static constexpr InstructionHint Ping{ Opcode::Ping, 0, 0, 0 };
		static constexpr InstructionHint Pong{ Opcode::Pong, 0, 0, 0 };

		static constexpr InstructionHint Locate{ Opcode::Locate, 0, 0, 0 };

		static constexpr InstructionHint Active{ Opcode::Active, 0, 0, 0 };
		static constexpr InstructionHint Inactive{ Opcode::Inactive, 0, 0, 0 };
		static constexpr InstructionHint Behavior{ Opcode::Behavior, 1, 1, 1 };
		static constexpr InstructionHint CalibrationGr{ Opcode::CalibrationGr, 4,  3, 12 };
		static constexpr InstructionHint CalibrationAc{ Opcode::CalibrationAc, 4, 12, 48 };
		static constexpr InstructionHint CalibrationMg{ Opcode::CalibrationMg, 4, 12, 48 };

		static constexpr InstructionHint Status{ Opcode::Status, 0, 0, 0 };
		static constexpr InstructionHint ImuRaw{ Opcode::ImuRaw, 0, 0, 0 };
		static constexpr InstructionHint ImuQuat{ Opcode::ImuQuat, 0, 0, 0 };
	};

}	// namespace dkvr