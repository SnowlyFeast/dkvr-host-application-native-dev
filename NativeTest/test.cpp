#include "pch.h"

#include "../DKVRHostNative/export/dkvr_host.h"

char global_buffer[2048];

TEST(Struct, size_test)
{
	// check header and dll version
	int assertion;
	dkvrAssertVersion(DKVR_HOST_EXPORTED_HEADER_VER, &assertion);
	ASSERT_TRUE(assertion);
	
	// create instance and assert handle
	DKVRHostHandle handle = nullptr;
	dkvrCreateInstance(&handle);
	ASSERT_NE(handle, nullptr);

	// run instance and assert
	dkvrRunHost(handle);
	dkvrIsRunning(handle, &assertion);
	ASSERT_TRUE(assertion);

	bool exit = false;
	while (!exit)
	{
		int input;
		std::cin >> input;

		switch (input)
		{
		case -1:
			exit = true;
			break;

		case 0:
		{
			dkvrLoggerGetUncheckedLogAll(handle, global_buffer, sizeof(global_buffer));
			std::cout << global_buffer;
		}

		case 1:
		{
			dkvrLoggerGetUncheckedLogOne(handle, global_buffer, sizeof(global_buffer));
			std::cout << global_buffer;
		}

		default:
			break;
		}
	}


	// clean up
	dkvrStopHost(handle);
	dkvrLoggerGetUncheckedLogAll(handle, global_buffer, sizeof(global_buffer));
	std::cout << global_buffer;

	dkvrDeleteInstance(&handle);
}

//
//using namespace std;
//using namespace dkvr;
//
//void Init();
//void OpenDat(const string, vector<Vector3>&);
//void SaveDat(const string, const vector<Vector3>&);
//void PrintMatrix(const Vector&);
//void PrintMatrix(const Matrix&);
//
//bool exit_flag = false;


//TEST(Matrix, TestName) {
//	Init();
//
//	vector<Vector3> error;
//	OpenDat("c:/error.dat", error);
//
//	float mean = 0;
//	for (const Vector3& v : error)
//		mean += sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
//	mean /= error.size();
//	
//	float var = 0;
//	for (const Vector3& v : error)
//		var += powf(sqrtf(v.x * v.x + v.y * v.y + v.z * v.z) - mean, 2);
//	var /= error.size();
//
//	vector<Vector3> samples;
//	vector<Vector3> new_cal(samples.size());
//	OpenDat("c:/original.dat", samples);
//	
//	EllipsoidParameter param = EllipsoidEstimator::EstimateEllipsoid(samples, var);
//	Vector cv = param.GetCenterVector();
//	Matrix tm = param.GetTransformationMatrix();
//
//	PrintMatrix(cv);
//	PrintMatrix(tm);
//
//	for (auto iter = samples.begin(); iter != samples.end(); iter++) {
//		new_cal.push_back(tm * (iter->ToVector() - cv));
//	}
//
//	vector<Vector3> calib;
//	OpenDat("c:/calib.dat", calib);
//
//	float tot = 0;
//	cout << "Original\t\tPre Cal.\t\tNew Cal." << endl;
//	for (int i = 0; i < calib.size(); i++) {
//		Vector3 v_diff = calib[i] - new_cal[i];
//		float f_diff = sqrtf(powf(v_diff.x, 2) + powf(v_diff.y, 2) + powf(v_diff.z, 2));
//		tot += f_diff;
//		cout << f_diff << endl;
//	}
//	SaveDat("./calib2.dat", new_cal);
//}
//
//
//void Init()
//{
//	cout.precision(3);
//}
//
//void OpenDat(const string path, vector<Vector3>& dat) 
//{
//	ifstream f(path);
//	string buffer;
//	queue<string> buff;
//	while (getline(f, buffer)) {
//		stringstream ss(buffer);
//		string temp;
//		while (getline(ss, temp, ' '))
//			buff.push(temp);
//	}
//	while (buff.size() >= 3) {
//		float x = stof(buff.front());
//		buff.pop();
//		float y = stof(buff.front());
//		buff.pop();
//		float z = stof(buff.front());
//		buff.pop();
//		dat.push_back(Vector3{ x, y, z });
//	}
//	f.close();
//}
//
//void SaveDat(const string path, const vector<Vector3>& dat)
//{
//	ofstream f(path);
//	for (Vector3 s : dat) {
//		f << s.x << " " << s.y << " " << s.z << endl;
//	}
//	f.close();
//}
//
//void PrintMatrix(const Vector& vec) 
//{
//	std::cout << "<";
//	for (size_t i = 0; i < vec.size(); i++)
//		std::cout << vec[i] << "\t";
//	std::cout << ">" << std::endl;
//}
//
//void PrintMatrix(const Matrix& mat)
//{
//	for (unsigned long i = 0; i < mat.row(); i++) {
//		for (unsigned long j = 0; j < mat.column(); j++) {
//			std::cout << mat[i][j] << "\t";
//		}
//		std::cout << std::endl;
//	}
//}