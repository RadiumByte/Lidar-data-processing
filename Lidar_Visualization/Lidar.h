///////////////////////////
// Anton Fedyashov, 2018 //
// Lidar data processing //
///////////////////////////

#include <opencv2\video\tracking.hpp>

#pragma once

#define WIN32_LEAN_AND_MEAN

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <math.h>
#include <algorithm>

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <gl\GL.h>

#include "KalmanFilter.h"

// Constants
#pragma region SETTINGS
// Math constants
#define M_PI 3.1415926535897932384626433832795

// Server settings
#define BUFFER 19
#define PORT 1234

// PointCloud settings
#define MAX_ROWS 1000

// Plane detection settings
#define CHECK_STEP 2
#define BORDER_ANGLE 30

// OpenGl settings
#define DEFAULT_SCALE    0.0015     // default scale of 3d surface in DrawSurfaceGL (default 1)
#define WIDTH_3D         6.0        // width of surface in DrawSurfaceGL            (default 0.5)
#define LENGTH_3D        6.0        // length of surface in DrawSurfaceGL           (default 0.5)
#define BASE_HEIGHT      0.0        // base height of blue horizontal surface       (default 0.0)
#define WALL_HEIGHT      100.0      // height of walls in 3D rendering
#define POINT_RADIUS     0.01 
#define POINT_RES        100

// Beacon detection settings
#define MAX_BEACON_RANGE     50    // max distance to beacons in cm  (default 100)

#pragma endregion

// Keyboard settings
#pragma region KEY_BINDS
#define UP_KEY     2490368
#define DOWN_KEY   2621440
#define LEFT_KEY   2424832
#define RIGHT_KEY  2555904
#define PLUS_KEY   61
#define MINUS_KEY  45
#define W_KEY      119
#define S_KEY      115
#define D_KEY      100
#define A_KEY      97
#define ESC_KEY    27

#pragma endregion

enum draw_dir            // view direction for correct rendering
{
	FRONT, LEFT, RIGHT, BACK
};

// raw data structure from lidar
struct Sample
{
	int angle;
	int range;
	int signal;
	double x;
	double y;
};

// 3D structure for parameter transfer into OpenGL callback
#pragma pack(push, 1)
struct gl_pack                 
{
	std::vector<cv::Mat_<cv::Point_<double>>> input;    // Objects to draw

	GLdouble scale;            // general scale of scene
	GLdouble angle_ud;         // up/down angle
	GLdouble angle_lr;         // left/right angle
	GLdouble size;             // horizontal size of surface
	draw_dir direction;        // observing direction
};
#pragma pack(pop)

// 2D structure for parameter transfer into OpenGL callback
#pragma pack(push, 1)
struct gl_pack_raw                 
{
	cv::Mat_<cv::Point_<double>> PointCloud;    // Objects to draw

	GLfloat scale;            // general scale of scene
};
#pragma pack(pop)

#pragma region Utility_functions
// Degrees to Radians
inline double ToRadians(const int millidegrees)
{
	const double dev_pi_180 = 0.017453292519943f;
	return ((double)millidegrees / 1000.0) * dev_pi_180;
}

// Radians to Degrees
inline double ToDegrees(const double radians)
{
	const double dev_180_pi = 57.295779513082320f;
	return radians * dev_180_pi;
}

// Scalar production of two vectors
inline double ScalarProduct(const cv::Point_<double> &first_vector, const cv::Point_<double> &second_vector)
{
	return first_vector.x * second_vector.x + first_vector.y * second_vector.y;
}

// Distance between two points
inline double DistBetween(const cv::Point_<double> &first_point, const cv::Point_<double> &second_point)
{
	return sqrt((second_point.x - first_point.x)*(second_point.x - first_point.x) + (second_point.y - first_point.y)*(second_point.y - first_point.y));
}

// Singular value decomposition of matrix
cv::Mat_<double> MatrixSVD(const cv::Mat_<double> &points);

double AngleBetweenVectors(const cv::Point_<double> &first_vector, const cv::Point_<double> &second_vector);

#pragma endregion

// Get objects (planes) from point cloud
std::vector<cv::Mat_<cv::Point_<double>>> GetObjects(const cv::Mat_<cv::Point_<double>> &PointCloud, const int cloud_size);

// Get locaton of beacons in raw point cloud
std::vector<cv::Point_<double>> GetBeacons(std::vector<Sample> &RawPointCloud);

// 3D rendering
void DrawPlatesGL(const std::vector<cv::Mat_<cv::Point_<double>>> &input, const std::string name, const GLdouble start_angle);
void on_opengl_plates(void* param);

// 2D rendering
void DrawRawGL(const cv::Mat_<cv::Point_<double>> &PointCloud, const std::string name);
void on_opengl_raw(void* param);