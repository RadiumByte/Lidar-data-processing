#include "KalmanFilter.h"

///////////// Kalman Filter ////////////////

void KalmanFilter::Init(cv::Mat &x_in, cv::Mat &P_in, cv::Mat &F_in,
	cv::Mat &H_in, cv::Mat &R_in, cv::Mat &Q_in) 
{
	x = x_in;
	P = P_in;
	F = F_in;
	H = H_in;
	R = R_in;
	Q = Q_in;
}

void KalmanFilter::Predict() 
{
	std::cout << "F matrix: " << std::endl;
	std::cout << F << std::endl;
	x = F * x;
	std::cout << "X matrix: " << std::endl;
	std::cout << x << std::endl;

	cv::Mat F_t = F.t();
	std::cout << "F_t matrix: " << std::endl;
	std::cout << F_t << std::endl;

	std::cout << "Q matrix: " << std::endl;
	std::cout << Q << std::endl;

	std::cout << "P matrix: " << std::endl;
	std::cout << P << std::endl;
	std::cout << "F matrix: " << std::endl;
	std::cout << F << std::endl;

	std::cout << "F * P matrix: " << std::endl;
	std::cout << F * P << std::endl;

	P = F * P * F_t + Q;
	std::cout << "P matrix: " << std::endl;
	std::cout << P << std::endl;
}

void KalmanFilter::Update(const cv::Mat &z)
{
	cv::Mat y = z - (H * x);
	std::cout << "Y matrix: " << std::endl;
	std::cout << y << std::endl;

	cv::Mat H_t = H.t();
	std::cout << "H_t matrix: " << std::endl;
	std::cout << H_t << std::endl;

	std::cout << std::endl << "Special test of variables..." << std::endl;
	std::cout << "H matrix: " << std::endl;
	std::cout << H << std::endl;
	std::cout << "P matrix: " << std::endl;
	std::cout << P << std::endl;
	std::cout << "R matrix: " << std::endl;
	std::cout << R << std::endl;

	cv::Mat S = H * P * H_t + R;
	std::cout << "S matrix: " << std::endl;
	std::cout << S << std::endl;

	cv::Mat S_i = S.inv();
	std::cout << "S_i matrix: " << std::endl;
	std::cout << S_i << std::endl;

	cv::Mat K = (P * H_t) * S_i;
	std::cout << "K matrix: " << std::endl;
	std::cout << K << std::endl;

	//new estimate
	x = x + (K * y);
	std::cout << "New estimate X matrix: " << std::endl;
	std::cout << x << std::endl;

	cv::Size s = x.size();
	int rows = s.height;
	int cols = s.width;

	int x_size = std::max(rows, cols);

	std::cout << "X size: " << x_size << std::endl;

	cv::Mat I = I.eye(x_size, x_size, CV_32F);
	std::cout << "I matrix: " << std::endl;
	std::cout << I << std::endl;

	P = (I - K * H) * P;
	std::cout << "P matrix: " << std::endl;
	std::cout << P << std::endl;
}

///////////// Extended Kalman Filter ////////////////

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
	is_initialized = false;

	prev_timestamp = 0;

	// initializing matrices
	//measurement covariance matrix
	float r_data[4] = { 0.0225, -0, 0, 0.0225};
	R_laser = cv::Mat(2, 2, CV_32F, r_data);

	float h_data[8] = { 1, 0, 0, 0, 0, 1, 0, 0 };
	H_laser = cv::Mat(2, 4, CV_32F, h_data);
	
	//set the acceleration noise components
	noise_ax = 9;
	noise_ay = 9;
}

void ExtendedKalmanFilter::ProcessData(const LaserData &data_pack) 
{
	/*****************************************************************************
	*  Initialization
	****************************************************************************/
	if (!is_initialized) 
	{
		// first measurement
		float x_data[4] = { data_pack.raw_measurements.at<float>(0, 0), data_pack.raw_measurements.at<float>(1, 0), 0, 0 };
		filter.x = cv::Mat(4, 1, CV_32F, x_data);
		std::cout << "First initialization. State vector X:" << std::endl;
		std::cout << filter.x << std::endl;
		/*
		if (fabs(filter.x.at<float>(0,0)) < EPS && fabs(filter.x.at<float>(1,0)) < EPS) 
		{
			filter.x.at<float>(0, 0) = EPS;
			filter.x.at<float>(1, 0) = EPS;
		}
		*/

		// Initial covariance matrix
		float p_data[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 100, 0, 0, 0, 0, 100 };
		filter.P = cv::Mat(4, 4, CV_32F, p_data);
		std::cout << "Initial covariance matrix P: " << std::endl;
		std::cout << filter.P << std::endl;

		// Store the initial timestamp for dt calculation
		prev_timestamp = data_pack.timestamp;
		is_initialized = true;

		return;
	}

	/*****************************************************************************
	*  Prediction
	****************************************************************************/

	double dt = (data_pack.timestamp - prev_timestamp);	//dt in seconds
	prev_timestamp = data_pack.timestamp;

	std::cout << "Delta time: " << dt << std::endl;

	//Modify the F matrix so that the time is integrated
	float f_data[16] = { 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1 };
	filter.F = cv::Mat(4, 4, CV_32F, f_data);
	std::cout << "F matrix: " << std::endl;
	std::cout << filter.F << std::endl;

	double dt_2 = dt * dt;
	double dt_3 = dt_2 * dt;
	double dt_4 = dt_3 * dt;

	//set the process covariance matrix Q
	float q_data[16] = { dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
		0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
		dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
		0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay };

	filter.Q = cv::Mat(4, 4, CV_32F, q_data);
	std::cout << "Covariance matrix Q: " << std::endl;
	std::cout << filter.Q << std::endl;

	std::cout << "Predict called..." << std::endl;
	filter.Predict();


	/*****************************************************************************
	*  Update
	****************************************************************************/

	// Laser updates
	filter.H = H_laser;
	filter.R = R_laser;
	filter.Update(data_pack.raw_measurements);
}