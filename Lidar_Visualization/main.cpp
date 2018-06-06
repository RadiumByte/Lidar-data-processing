////////////////////////////////
//	 Anton Fedyashov, 2018	  //
//   Lidar data processing    //
////////////////////////////////

#include "Lidar.h"
#include "KalmanFilter.h"
#include <conio.h>

#pragma comment (lib, "Ws2_32.lib")

int main()
{
	ExtendedKalmanFilter filter;

	LaserData data;

	data.timestamp = 1;
	data.raw_measurements = cv::Mat(2, 1, CV_32F);

	float x = 32.453;
	float y = 78.123;
	std::cout << "Input values: " << x << " : " << y << std::endl;

	data.raw_measurements.at<float>(0, 0) = x;
	data.raw_measurements.at<float>(1, 0) = y;

	std::cout << "Process data called..." << std::endl;
	filter.ProcessData(data);

	x = 33.178;
	y = 76.945;
	data.timestamp += 2;
	data.raw_measurements.at<float>(0, 0) = x;
	data.raw_measurements.at<float>(1, 0) = y;
	std::cout << "Input values: " << x << " : " << y << std::endl;

	std::cout << "Process data called..." << std::endl;
	filter.ProcessData(data);
	float new_x = filter.filter.x.at<float>(0, 0);
	float new_y = filter.filter.x.at<float>(1, 0);
	std::cout << new_x << " : " << new_y << std::endl;

	system("pause");
	/*
	SOCKET s;
	struct sockaddr_in server, si_other;
	int slen, recv_len;
	char buf[BUFFER];
	WSADATA wsa;

	slen = sizeof(si_other);

	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		std::cout << "Failed. Error Code: " << WSAGetLastError() << std::endl;
		exit(EXIT_FAILURE);
	}

	if ((s = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
		std::cout << "Could not create socket: " << WSAGetLastError() << std::endl;
	
	std::cout << "Socket created." << std::endl;
	
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(PORT);

	if (bind(s, (struct sockaddr *)&server, sizeof(server)) == SOCKET_ERROR)
	{
		printf("Bind failed with error code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	std::cout << "Bind done" << std::endl;
	
	std::vector<Sample> RawPointCloud;
	ExtendedKalmanFilter lidar_kalman;

	int angle_degrees = 0;
	int angle_degrees_prev = 0;
	long long time = 0;

	try
	{
		do
		{
			// Key pressing checking
			if (_kbhit())
				break;

			memset(buf, '\0', BUFFER);

			if ((recv_len = recvfrom(s, buf, BUFFER, 0, (struct sockaddr *) &si_other, &slen)) == SOCKET_ERROR)
			{
				printf("recvfrom() failed with error code : %d", WSAGetLastError());
				exit(EXIT_FAILURE);
			}

			// Parsing of incoming data
			if (buf[0] == '(')
			{
				time++;

				std::string data(buf);

				if (data == "(HALT)")
					break;

				//std::cout << data << std::endl;

				// Parsing of millidegrees
				int angle_degrees = stoi(data.substr(1, 7));

				// Current 360 deg. scan is done, new scan incoming
				if (angle_degrees < angle_degrees_prev)
				{
					std::vector<cv::Point_<double>> Beacons = GetBeacons(RawPointCloud);

					float new_x = 0;
					float new_y = 0;

					if (!Beacons.empty())
					{
						LaserData current_data;
						cv::Point_<double> center = Beacons.back();

						current_data.raw_measurements = cv::Mat(2, 1, CV_32F);
						current_data.raw_measurements.at<float>(0, 0) = center.x;
						current_data.raw_measurements.at<float>(1, 0) = center.y;
						current_data.timestamp = time;

						lidar_kalman.ProcessData(current_data);

						new_x = lidar_kalman.filter.x.at<float>(0, 0);
						new_y = lidar_kalman.filter.x.at<float>(1, 0);
					}

					for (cv::Point_<double> current : Beacons)
					{
						std::cout << "X: " << current.x << " || " << "Y: " << current.y << std::endl;
						std::cout << "X: " << new_x << " || " << "Y: " << new_y << std::endl;
					}
					std::cout << "-------------END-OF-SCAN-----------------" << std::endl;

					RawPointCloud.clear();
					//std::vector<cv::Mat_<cv::Point_<double>>> Objects = GetObjects(PointCloud, current_row + 1);

	*/				//DrawRawGL(PointCloud, "Current location");

					// Clearing of PointCloud
					/*
					for (int i = 0; i <= current_row; i++)
					{
						PointCloud(current_row).x = 0;
						PointCloud(current_row).y = 0;
					}
					current_row = 0;
					*/
	/*			}
				angle_degrees_prev = angle_degrees;

				// Parsing of angle
				int range = stoi(data.substr(8, 13));

				// Parsing of signal strength
				int signal = stoi(data.substr(14, data.length() - 1));

				if (signal > 50)
				{
					double angle_radians = ToRadians(angle_degrees);

					// Converting from polar coords to cartesian
					double x = range * cos(angle_radians);
					double y = range * sin(angle_radians);

					if (range > 5)
					{
						Sample current;
						current.angle = angle_degrees;
						current.range = range;
						current.signal = signal;
						current.x = x;
						current.y = y;

						RawPointCloud.push_back(current);
					}
				}
			}
		} while (true);
	}
	catch (std::exception e)
	{
		std::cout << e.what() << std::endl;
	}

	std::cout << "Server is shutting down...";

	closesocket(s);
	WSACleanup();

	return 0;
	*/
}
