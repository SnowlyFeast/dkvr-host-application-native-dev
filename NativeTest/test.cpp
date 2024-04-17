#include "pch.h"

#include "export/dkvr_host.h"

// main component
DKVRHostHandle handle = nullptr;
char global_buffer[2048];
std::atomic_bool exit_flag = false;

// imu reader
std::atomic_bool imu_read_exit = false;
std::unique_ptr<std::thread> imu_read_thread(nullptr);

// protos
void LoggerCheckingThreadLoop();
void ParseKeyInput();
void ReadIMUThreadLoop(int target, int interval);

TEST(Struct, size_test)
{
	// check header and dll version
	int assertion;
	dkvrAssertVersion(DKVR_HOST_EXPORTED_HEADER_VER, &assertion);
	ASSERT_TRUE(assertion);
	
	// create instance and assert handle
	dkvrCreateInstance(&handle);
	ASSERT_NE(handle, nullptr);

	// run instance and assert
	dkvrRunHost(handle);
	dkvrIsRunning(handle, &assertion);
	ASSERT_TRUE(assertion);

	// run local thread
	std::unique_ptr<std::thread> thread_console = std::make_unique<std::thread>(LoggerCheckingThreadLoop);


	// key input parser
	while (!exit_flag)
	{
		ParseKeyInput();
	}

	// close threads
	exit_flag = true;
	thread_console->join();

	// clean up
	dkvrStopHost(handle);
	dkvrLoggerGetUncheckedLogAll(handle, global_buffer, sizeof(global_buffer));
	std::cout << global_buffer;

	dkvrDeleteInstance(&handle);
}

void LoggerCheckingThreadLoop()
{
	while (!exit_flag)
	{
		int count = 0;
		dkvrLoggerGetUncheckCount(handle, &count);

		if (count)
		{
			dkvrLoggerGetUncheckedLogAll(handle, global_buffer, sizeof(global_buffer));
			std::cout << '\r' << global_buffer << "> ";
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
}

static bool compare(std::string& target, const char* ref)
{
	return !target.compare(ref);
}

static std::vector<std::string> split(std::string str, char Delimiter) {
	std::istringstream iss(str);
	std::string buffer;

	std::vector<std::string> result;

	while (getline(iss, buffer, Delimiter)) {
		result.push_back(buffer);
	}
	return result;
}

#define ASSERT_ARG(arg, ss)	if (arg.size() < ss) { std::cout << "Missing arguments." << std::endl; return; }
#define PARSE_STR(result, str, func)							\
do {															\
	try {														\
		result = func(str);										\
	}															\
	catch (std::invalid_argument except)						\
	{															\
		std::cout << "argument parsing failed." << std::endl;	\
	}															\
} while(0)

void ParseKeyInput()
{
	std::cout << "> ";
	std::string str;
	std::getline(std::cin, str, '\n');

	std::vector<std::string> arg = split(str, ' ');
	std::string cmd = arg[0];

	if (compare(cmd, "help"))
	{
		std::cout << "----------------------Available command----------------------" << std::endl;
		std::cout << "exit" << std::endl;
		std::cout << "list" << std::endl;
		std::cout << "behavior [index] [value]" << std::endl;
		std::cout << "imu [index] [interval]" << std::endl;
		std::cout << "imustop" << std::endl << std::endl;
		std::cout << "status" << std::endl;
		std::cout << "status update" << std::endl;
		std::cout << "locate [index]" << std::endl;

		std::cout << "calib status" << std::endl;
		std::cout << "calib begin [index]" << std::endl;
		std::cout << "calib continue" << std::endl;
		std::cout << "calib abort" << std::endl;

		std::cout << "save calib [index] [filename]" << std::endl;
		std::cout << "load calib [index] [filename]" << std::endl;
		std::cout << "-------------------------------------------------------------" << std::endl;
	}
	else if (compare(cmd, "exit"))
	{
		imu_read_exit = true;
		if (imu_read_thread)
			imu_read_thread->join();
		exit_flag = true;
	}
	else if (compare(cmd, "list"))
	{
		int count = 0;
		dkvrTrackerGetCount(handle, &count);

		std::cout << "idx\taddress\t\tconnection\tbehavior" << std::endl;
		std::cout << "-------------------------------------------------------------" << std::endl;
		for (int i = 0; i < count; i++)
		{
			long address = 0;
			dkvrTrackerGetAddress(handle, i, &address);

			int con = 0;
			dkvrTrackerGetConnectionStatus(handle, i, &con);
			std::string con_str = con == 0 ? "Disconnected" : (con == 1 ? "Handshaked" : "Connected");

			int active = 0;
			int raw = 0;
			int led = 0;
			dkvrTrackerGetActive(handle, i, &active);
			dkvrTrackerGetRaw(handle, i, &raw);
			dkvrTrackerGetLed(handle, i, &led);
			int behavior = (led << 2) | (raw << 1) | active;

			unsigned char* ptr = reinterpret_cast<unsigned char*>(&address);
			std::cout << i << '\t' << (int)ptr[0] << "." << (int)ptr[1] << "." << (int)ptr[2] << "." << (int)ptr[3] << '\t' << con_str << '\t'
				<< "[" << (active ? "active|" : "") << (raw ? "raw|" : "") << (led ? "led" : "") << "]"
				<< std::endl;
		}
	}
	else if (compare(cmd, "behavior"))
	{
		int target, behavior;
		ASSERT_ARG(arg, 3);
		PARSE_STR(target, arg[1], stoi);
		PARSE_STR(behavior, arg[2], stoi);

		int active = behavior & 0b001;
		int raw = behavior & 0b010;
		int led = behavior & 0b100;
		dkvrTrackerSetActive(handle, target, active);
		dkvrTrackerSetRaw(handle, target, raw);
		dkvrTrackerSetLed(handle, target, led);
		std::cout << "Setting " << target << "th tracker's behavior to " << behavior << "." << std::endl;
	}
	else if (compare(cmd, "imu"))
	{
		ASSERT_ARG(arg, 3);

		if (imu_read_thread)
		{
			std::cout << "IMU reader thread already running." << std::endl;
			return;
		}

		int target, interval;
		PARSE_STR(target, arg[1], stoi);
		PARSE_STR(interval, arg[2], stoi);

		imu_read_exit = false;
		imu_read_thread = std::make_unique<std::thread>(ReadIMUThreadLoop, target, interval);
	}
	else if (compare(cmd, "imustop"))
	{
		imu_read_exit = true;
		if (imu_read_thread)
			imu_read_thread->join();
		imu_read_thread.reset();
	}
	else if (compare(cmd, "status"))
	{
		int count = 0;
		dkvrTrackerGetCount(handle, &count);
		if (arg.size() >= 2 && compare(arg[1], "update"))
		{
			for (int i = 0; i < count; i++)
				dkvrTrackerRequestStatus(handle, i);
			std::cout << "Tracker status update requested." << std::endl;
		}
		else
		{
			std::cout << "idx\tinit\tlast error\tbattery %" << std::endl;
			std::cout << "-------------------------------------------------------------" << std::endl;
			for (int i = 0; i < count; i++)
			{
				int init, last_err, battery_perc;
				dkvrTrackerGetInitResult(handle, i, &init);
				dkvrTrackerGetLastError(handle, i, &last_err);
				dkvrTrackerGetBatteryPerc(handle, i, &battery_perc);
				std::cout << i << '\t' << std::hex << (bool)init << '\t' << last_err << "\t\t" << std::dec << battery_perc << std::endl;
			}
		}
	}
	else if (compare(cmd, "locate"))
	{
		int target;
		ASSERT_ARG(arg, 2);
		PARSE_STR(target, arg[1], stoi);

		dkvrTrackerRequestLocate(handle, target);
	}
	else if (compare(cmd, "calib"))
	{
		ASSERT_ARG(arg, 2);

		std::string& second = arg[1];
		if (compare(second, "status"))
		{
			int status = 0;
			char buffer[64]{};
			dkvrCalibratorGetStatus(handle, &status);
			dkvrCalibratorGetStatusString(handle, buffer, sizeof(buffer));
			std::cout << "Calibrator Status : " << buffer << std::endl;

			if (status)
			{
				int target = 0;
				dkvrCalibratorGetCurrentTarget(handle, &target);
				std::cout << "Current calibration target : " << target << std::endl;
			}

			if (status == 2)
			{
				dkvrCalibratorGetRequiredSampleTypeString(handle, buffer, sizeof(buffer));
				std::cout << "Required sample type : " << buffer << std::endl;
			}
		}
		else if (compare(second, "begin"))
		{
			int target;
			ASSERT_ARG(arg, 3);
			PARSE_STR(target, arg[2], stoi);

			dkvrCalibratorBeginWith(handle, target);
		}
		else if (compare(second, "continue"))
		{
			dkvrCalibratorContinue(handle);
		}
		else if (compare(second, "abort"))
		{
			dkvrCalibratorAbort(handle);
		}
		else
		{
			std::cout << "Unknown calibrator command." << std::endl;
		}
	}
	else if (compare(cmd, "save")) 
	{
		int target;
		ASSERT_ARG(arg, 4);
		PARSE_STR(target, arg[2], stoi);

		const void* ptr = nullptr;
		size_t count = 0;
		if (compare(arg[1], "calib"))
		{
			static Calibration calib;
			dkvrTrackerGetCalibration(handle, target, &calib);
			ptr = &calib;
			count = sizeof Calibration;
		}

		if (ptr && count) {
			std::ofstream output(arg[3], std::ios::binary | std::ios::trunc);
			if (output.is_open())
			{
				output.write(reinterpret_cast<const char*>(ptr), count);
				output.close();
				std::cout << "Calibration struct saved." << std::endl;
			} 
			else
			{
				std::cout << "Output file open failed." << std::endl;
			}
		}
		else
		{
			std::cout << "Save target is not specified." << std::endl;
		}
		
	}
	else if (compare(cmd, "load"))
	{
		int target;
		ASSERT_ARG(arg, 4);
		PARSE_STR(target, arg[2], stoi);

		std::ifstream input(arg[3], std::ios::binary);
		if (input.is_open())
		{
			input.seekg(0, input.end);
			int size = input.tellg();
			input.seekg(0, input.beg);

			char* buffer = new char[size];
			input.read(buffer, size);

			if (compare(arg[1], "calib"))
			{
				if (size == sizeof Calibration)
				{
					dkvrTrackerSetCalibration(handle, target, reinterpret_cast<Calibration*>(buffer));
					std::cout << "Calibration struct loaded." << std::endl;
				}
				else
				{
					std::cout << "filesize is not matching with sizeof Calibration struct." << std::endl;
				}
			}

			delete[] buffer;
		}
		else
		{
			std::cout << "Input file open failed." << std::endl;
		}
	}
	else
	{
		std::cout << "unknown command" << std::endl;
	}
}

void ReadIMUThreadLoop(int target, int interval)
{
	while (!imu_read_exit)
	{
		Quaternion quat{};
		Vector3 gyro{}, accel{}, mag{};
		dkvrTrackerGetQuat(handle, target, &quat);
		dkvrTrackerGetGyro(handle, target, &gyro);
		dkvrTrackerGetAccel(handle, target, &accel);
		dkvrTrackerGetMag(handle, target, &mag);
		
		std::cout << std::setprecision(3) << std::fixed << "Quat  : " << quat.x << "\t" << quat.y << "\t" << quat.z << "\t" << quat.w << std::endl;
		std::cout << std::setprecision(3) << std::fixed << "Gyro  : " << gyro.x << "\t" << gyro.y << "\t" << gyro.z << std::endl;
		std::cout << std::setprecision(3) << std::fixed << "Accel : " << accel.x << "\t" << accel.y << "\t" << accel.z << std::endl;
		std::cout << std::setprecision(3) << std::fixed << "Mag   : " << mag.x << "\t" << mag.y << "\t" << mag.z << std::endl;

		std::this_thread::sleep_for(std::chrono::milliseconds(interval));
	}
}