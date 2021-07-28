
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Dense>
#include "System.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;
string sData_path = "/home/dataset/EuRoC/MH-05/mav0/";
string sConfig_path = "../config/";

std::shared_ptr<System> pSystem;

void PubImuData()
{
	// change the path
	// string sImu_data_file = sData_path + "imu_pose.txt";
	string sImu_data_file = "../data/imu_pose_noise.txt";
	// string sImu_data_file = "/mnt/hgfs/bwedu/VIO/exercise/ex02/course2_hw_new/vio_data_simulation/bin/imu_pose_noise.txt";
	cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
	ifstream fsImu;
	fsImu.open(sImu_data_file.c_str());
	if (!fsImu.is_open())
	{
		cerr << "Failed to open imu file! " << sImu_data_file << endl;
		return;
	}

	std::string sImu_line;
	double dStampNSec = 0.0;
	Vector3d vAcc;
	Vector3d vGyr;
	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
	{
		std::istringstream ssImuData(sImu_line);
		// imu data format: timestamp (1)，imu quaternion(4)，imu position(3)，imu gyro(3)，imu acc(3)
		// we only need timestamp(1), gyro(3), acc(3)
		ssImuData >> dStampNSec;
		double ignored; //ignore quarenion(4),position(3)
		for (size_t i = 0; i < 7; ++i)
			ssImuData >> ignored;
		ssImuData >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
		// cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
		pSystem->PubImuData(dStampNSec, vGyr, vAcc);
		usleep(5000 * nDelayTimes);
	}
	fsImu.close();
}

void PubImageData()
{
	// open cam_pose.txt only for the timestamps
	string sImage_file = "../data/cam_pose.txt";
	// string sImage_file = "/mnt/hgfs/bwedu/VIO/exercise/ex02/course2_hw_new/vio_data_simulation/bin/cam_pose.txt";
	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;
	ifstream fsImage;
	fsImage.open(sImage_file.c_str());
	if (!fsImage.is_open())
	{
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}
	std::string sImage_line;
	double dStampNSec;
	string sImgFileName;
	int n = 0; // read all_points at each dStampNSec

	// cv::namedWindow("SOURCE IMAGE", CV_WINDOW_AUTOSIZE);
	// read the timestamp from cam_pose.txt
	while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
	{
		std::istringstream ssImgData(sImage_line);
		// only read the timestamp
		ssImgData >> dStampNSec;
		cout << "cam time: " << fixed << dStampNSec << endl;
		string all_points_file_name = "../data/keyframe/all_points_" + to_string(n) + ".txt";
		// string filePath = "/mnt/hgfs/bwedu/VIO/exercise/ex02/course2_hw_new/vio_data_simulation/bin/";
		// string all_points_file_name = filePath + "keyframe/all_points_" + to_string(n) + ".txt";
		cout << "points_file: " << all_points_file_name << endl;

		vector<Point2f> FeaturePoints;
		ifstream fsImgPts;
		fsImgPts.open(all_points_file_name);
		if (!fsImgPts.is_open())
		{
			cerr << "Failed to open image file! " << all_points_file_name << endl;
		}

		string sImgPts_line;

		// read feature points from all_points.txt
		while (getline(fsImgPts, sImgPts_line) && !sImgPts_line.empty())
		{
			istringstream ssImgPoint(sImgPts_line);
			// file format in each line: x, y, z, 1, u, v
			// we only need u, v
			double ignored;
			for (size_t i = 0; i < 4; ++i)
			{
				ssImgPoint >> ignored;
			}
			Point2f feature_point;
			ssImgPoint >> feature_point.x;
			ssImgPoint >> feature_point.y;

			FeaturePoints.push_back(feature_point);

		}

		pSystem->PubImageData(dStampNSec, FeaturePoints);

		usleep(50000 * nDelayTimes);
		n++;
	}
	fsImage.close();
}

#if 0
void PubImuData()
{
	string sImu_data_file = sConfig_path + "MH_05_imu0.txt";
	cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
	ifstream fsImu;
	fsImu.open(sImu_data_file.c_str());
	if (!fsImu.is_open())
	{
		cerr << "Failed to open imu file! " << sImu_data_file << endl;
		return;
	}

	std::string sImu_line;
	double dStampNSec = 0.0;
	Vector3d vAcc;
	Vector3d vGyr;
	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
	{
		std::istringstream ssImuData(sImu_line);
		ssImuData >> dStampNSec >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
		// cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
		pSystem->PubImuData(dStampNSec / 1e9, vGyr, vAcc);
		usleep(5000*nDelayTimes);
	}
	fsImu.close();
}

void PubImageData()
{
	string sImage_file = sConfig_path + "MH_05_cam0.txt";

	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

	ifstream fsImage;
	fsImage.open(sImage_file.c_str());
	if (!fsImage.is_open())
	{
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampNSec;
	string sImgFileName;
	
	// cv::namedWindow("SOURCE IMAGE", CV_WINDOW_AUTOSIZE);
	while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
	{
		std::istringstream ssImuData(sImage_line);
		ssImuData >> dStampNSec >> sImgFileName;
		// cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;
		string imagePath = sData_path + "cam0/data/" + sImgFileName;

		Mat img = imread(imagePath.c_str(), 0);
		if (img.empty())
		{
			cerr << "image is empty! path: " << imagePath << endl;
			return;
		}
		pSystem->PubImageData(dStampNSec / 1e9, img);
		// cv::imshow("SOURCE IMAGE", img);
		// cv::waitKey(0);
		usleep(50000*nDelayTimes);
	}
	fsImage.close();
}
#endif

#ifdef __APPLE__
// support for MacOS
void DrawIMGandGLinMainThrd()
{
	string sImage_file = sConfig_path + "MH_05_cam0.txt";

	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

	ifstream fsImage;
	fsImage.open(sImage_file.c_str());
	if (!fsImage.is_open())
	{
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampNSec;
	string sImgFileName;

	pSystem->InitDrawGL();
	while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
	{
		std::istringstream ssImuData(sImage_line);
		ssImuData >> dStampNSec >> sImgFileName;
		// cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;
		string imagePath = sData_path + "cam0/data/" + sImgFileName;

		Mat img = imread(imagePath.c_str(), 0);
		if (img.empty())
		{
			cerr << "image is empty! path: " << imagePath << endl;
			return;
		}
		//pSystem->PubImageData(dStampNSec / 1e9, img);
		cv::Mat show_img;
		cv::cvtColor(img, show_img, CV_GRAY2RGB);
		if (SHOW_TRACK)
		{
			for (unsigned int j = 0; j < pSystem->trackerData[0].cur_pts.size(); j++)
			{
				double len = min(1.0, 1.0 * pSystem->trackerData[0].track_cnt[j] / WINDOW_SIZE);
				cv::circle(show_img, pSystem->trackerData[0].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
			}

			cv::namedWindow("IMAGE", CV_WINDOW_AUTOSIZE);
			cv::imshow("IMAGE", show_img);
			// cv::waitKey(1);
		}

		pSystem->DrawGLFrame();
		usleep(50000 * nDelayTimes);
	}
	fsImage.close();
}
#endif

int main(int argc, char **argv)
{
	if (argc != 3)
	{
		cerr << "./run_euroc PATH_TO_FOLDER/MH-05/mav0 PATH_TO_CONFIG/config \n"
			 << "For example: ./run_euroc /home/stevencui/dataset/EuRoC/MH-05/mav0/ ../config/" << endl;
		return -1;
	}
	sData_path = argv[1];
	sConfig_path = argv[2];

	pSystem.reset(new System(sConfig_path));

	std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);

	// sleep(5);
	std::thread thd_PubImuData(PubImuData);

	std::thread thd_PubImageData(PubImageData);

#ifdef __linux__
	std::thread thd_Draw(&System::Draw, pSystem);
#elif __APPLE__
	DrawIMGandGLinMainThrd();
#endif

	thd_PubImuData.join();
	thd_PubImageData.join();

	// thd_BackEnd.join();
	// thd_Draw.join();

	cout << "main end... see you ..." << endl;
	return 0;
}
