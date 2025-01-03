#pragma once

#include <chrono>
#include <ctime>
#include <list>
#include <mutex>
#include <ostream>
#include <queue>
#include <string>
#include <vector>

#include "fmt/chrono.h"
#include "fmt/core.h"

#ifndef DKVR_LOGGER_GLOBAL_LEVEL
#	define DKVR_LOGGER_GLOBAL_LEVEL	2
#endif

#if (DKVR_LOGGER_GLOBAL_LEVEL < 1)
#	define DKVR_LOGGER_SUPPRESS_INFO
#endif
#if (DKVR_LOGGER_GLOBAL_LEVEL < 2)
#	define DKVR_LOGGER_SUPPRESS_DEBUG
#endif

namespace dkvr {

	/// <summary>
	/// Some stupid logging class for internal usage.
	/// </summary>
	class Logger
	{
	public:
		__pragma(dkvr_export) enum class Level {
			Error,
			Info,
			Debug
		};

		__pragma(dkvr_export) enum class Mode {
			Echo,	// print logs to the console immediately
			Burst,	// hold logs ultil explicitly call PrintUnchecked()
			Silent	// all logs will automatically checked (ignored)
		};

		static Logger& GetInstance();

		template<typename... Args>
		static std::string FormatString(const fmt::format_string<Args...> fmt, Args&&... args);

		template<typename... Args>
		void Error(const fmt::format_string<Args...> fmt, Args&&... args);
		template<typename... Args>
		void Info(const fmt::format_string<Args...> fmt, Args&&... args);
		template<typename... Args>
		void Debug(const fmt::format_string<Args...> fmt, Args&&... args);

		int GetUncheckedCount() const;
		void PrintUnchecked();
		void PrintUnchecked(std::size_t count);

		Logger& operator<< (std::string&& str) { Push(std::move(str)); return *this; }
		Logger& operator<< (const std::string& str) { Push(str); return *this; }
		Logger& operator<< (Level level) { set_level(level); return *this; }
		Logger& operator<< (Mode mode) { set_mode(mode); return *this; }

		std::ostream& ostream() const { return *out_; }
		Level level() const { return level_; }
		Mode mode() const { return mode_; }

		void set_ostream(std::ostream& ostream) { out_ = &ostream; }
		void set_level(Level level) { level_ = level; }
		void set_mode(Mode mode) { mode_ = mode; }

	private:
		Logger();
		Logger(const Logger&) = delete;
		Logger(Logger&&) = delete;
		virtual ~Logger();
		void operator= (const Logger&) = delete;
		void operator= (Logger&&) = delete;

		void Push(std::string&& str);
		void Push(const std::string& str);

		mutable std::mutex mutex_;
		std::queue<std::string> unchecked_;
		std::ostream* out_;
		Level level_;
		Mode mode_;
	};

	template<typename... Args>
	static std::string Logger::FormatString(const fmt::format_string<Args...> fmt, Args&&... args)
	{
		return fmt::vformat(fmt.get(), fmt::make_format_args(args...));
	}

	template<typename... Args>
	inline void Logger::Error(const fmt::format_string<Args...> fmt, Args&&... args)
	{
		if (level_ >= Level::Error) {
			std::string msg = fmt::vformat(fmt.get(), fmt::make_format_args(args...));
			Push(fmt::format("[ERROR] {:%Y-%m-%d %H:%M:%S}\t{}", fmt::localtime(std::time(nullptr)), msg));
		}
	}

	template<typename... Args>
	inline void Logger::Info(const fmt::format_string<Args...> fmt, Args&&... args)
	{
#ifndef DKVR_LOGGER_SUPPRESS_INFO
		if (level_ >= Level::Info) {
			std::string msg = fmt::vformat(fmt.get(), fmt::make_format_args(args...));
			Push(fmt::format("[INFO] {:%Y-%m-%d %H:%M:%S}\t{}", fmt::localtime(std::time(nullptr)), msg));
		}
#endif
	}

	template<typename... Args>
	inline void Logger::Debug(const fmt::format_string<Args...> fmt, Args&&... args)
	{
#ifndef DKVR_LOGGER_SUPPRESS_DEBUG
		if (level_ >= Level::Debug) {
			std::string msg = fmt::vformat(fmt.get(), fmt::make_format_args(args...));
			Push(fmt::format("[DEBUG] {:%Y-%m-%d %H:%M:%S}\t{}", fmt::localtime(std::time(nullptr)), msg));
		}
#endif
	}

}	// namespace dkvr