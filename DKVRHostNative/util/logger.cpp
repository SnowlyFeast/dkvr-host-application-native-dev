#include "logger.h"

#include <iostream>
#include <string>
#include <string_view>

namespace dkvr {

	template <typename T>
	constexpr auto type_name() {
		std::string_view name, prefix, suffix;
#ifdef __clang__
		name = __PRETTY_FUNCTION__;
		prefix = "auto type_name() [T = ";
		suffix = "]";
#elif defined(__GNUC__)
		name = __PRETTY_FUNCTION__;
		prefix = "constexpr auto type_name() [with T = ";
		suffix = "]";
#elif defined(_MSC_VER)
		name = __FUNCSIG__;
		prefix = "auto __cdecl type_name<";
		suffix = ">(void)";
#endif
		name.remove_prefix(prefix.size());
		name.remove_suffix(suffix.size());
		return name;
	}

	Logger& Logger::GetInstance()
	{
		static Logger instance;
		return instance;
	}

	Logger::Logger() : mutex_(), unchecked_(), checked_(), out_(&std::cout), level_(Level::Info), mode_(Mode::Burst) { }

	Logger::~Logger()
	{

	}

	void Logger::PrintUnchecked()
	{
		PrintUnchecked(unchecked_.size());
	}

	void Logger::PrintUnchecked(std::size_t count)
	{
		std::lock_guard<std::mutex> lock(mutex_);

		if (unchecked_.size() < count)
			count = unchecked_.size();

		for (std::size_t i = 0; i < count; i++) {
			std::string& str = unchecked_.front();
			checked_.push_back(str);
			*out_ << str << std::endl;
			unchecked_.pop();
		}
	}

	void Logger::PrintChecked(std::size_t count)
	{
		PrintChecked(0, count);
	}

	void Logger::PrintChecked(std::size_t from, std::size_t count)
	{
		std::lock_guard<std::mutex> lock(mutex_);

		// TODO: load log from save file

	}

	void Logger::Push(std::string&& str)
	{
		std::lock_guard<std::mutex> lock(mutex_);

		switch (mode_)
		{
		case Logger::Mode::Echo:
			(*out_) << str << std::endl;
			checked_.emplace_back(std::move(str));
			break;

		default:
		case Logger::Mode::Burst:
			unchecked_.emplace(std::move(str));
			break;

		case Logger::Mode::Silent:
			checked_.emplace_back(std::move(str));
			break;
		}
	}

	void Logger::Push(const std::string& str)
	{
		std::lock_guard<std::mutex> lock(mutex_);

		switch (mode_)
		{
		case Logger::Mode::Echo:
			(*out_) << str << std::endl;
			checked_.push_back(str);
			break;

		default:
		case Logger::Mode::Burst:
			unchecked_.push(str);
			break;

		case Logger::Mode::Silent:
			checked_.push_back(str);
			break;
		}
	}

}	// namespace dkvr