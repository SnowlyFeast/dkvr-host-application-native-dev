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
			std::cout << global_buffer;
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

void ParseKeyInput()
{
	std::string str;
	std::getline(std::cin, str, '\n');

	std::vector<std::string> arg = split(str, ' ');
	std::string cmd = arg[0];

	if (compare(cmd, "help"))
	{
		std::cout << "exit" << std::endl;
		std::cout << "list" << std::endl;
		std::cout << "behavior [index] [value]" << std::endl;
		std::cout << "imu [index] [interval]" << std::endl;
		std::cout << "imustop" << std::endl << std::endl;

		std::cout << "calib status" << std::endl;
		std::cout << "calib begin [index]" << std::endl;
		std::cout << "calib continue" << std::endl;
		std::cout << "calib abort" << std::endl;
	}
	else if (compare(cmd, "exit")) 
	{
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
		if (arg.size() < 3)
		{
			std::cout << "Missing arguments." << std::endl;
			return;
		}

		int target;
		int behavior;
		try {
			target = stoi(arg[1]);
			behavior = stoi(arg[2]);
		}
		catch (std::invalid_argument inv_arg)
		{
			std::cout << "argument parsing failed." << std::endl;
		}
		
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
		if (arg.size() < 3)
		{
			std::cout << "Missing arguments." << std::endl;
			return;
		}

		if (imu_read_thread)
		{
			std::cout << "IMU reader thread already running." << std::endl;
			return;
		}

		int target, interval;
		try
		{
			target = stoi(arg[1]);
			interval = stoi(arg[2]);
		}
		catch (std::invalid_argument inv_arg)
		{
			std::cout << "argument parsing failed." << std::endl;
		}
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
	else if (compare(cmd, "calib"))
	{
		if (arg.size() < 2)
		{
			std::cout << "Missing arguments." << std::endl;
			return;
		}

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
			if (arg.size() < 3)
			{
				std::cout << "Missing arguments." << std::endl;
				return;
			}
			int target;
			try
			{
				target = stoi(arg[2]);
			}
			catch (std::invalid_argument inv_arg)
			{
				std::cout << "argument parsing failed." << std::endl;
			}
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

		std::cout << "Quat  : " << quat.x << "\t" << quat.y << "\t" << quat.z << "\t" << quat.w << std::endl;
		std::cout << "Gyro  : " << gyro.x << "\t" << gyro.y << "\t" << gyro.z << std::endl;
		std::cout << "Accel : " << accel.x << "\t" << accel.y << "\t" << accel.z << std::endl;
		std::cout << "Mag   : " << mag.x << "\t" << mag.y << "\t" << mag.z << std::endl;

		std::this_thread::sleep_for(std::chrono::milliseconds(interval));
	}
}