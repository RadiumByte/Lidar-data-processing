#pragma once
#include <opencv2/core.hpp>
#include <algorithm>
#include <iostream>

#define EPS 0.0001

class LaserData 
{
public:
	long long timestamp;

	cv::Mat raw_measurements;
};

class KalmanFilter 
{
public:
	// state vector
	cv::Mat x;

	// state covariance matrix
	cv::Mat P;
	
	// state transition matrix
	cv::Mat F;
	
	// process covariance matrix
	cv::Mat Q;
	
	// measurement matrix
	cv::Mat H;
	
	// measurement covariance matrix
	cv::Mat R;
	
	KalmanFilter() {};
	~KalmanFilter() {};

	/**
	* Init Initializes Kalman filter
	* x_in - Initial state
	* P_in - Initial state covariance
	* F_in - Transition matrix
	* H_in - Measurement matrix
	* R_in - Measurement covariance matrix
	* Q_in - Process covariance matrix
	*/
	void Init(cv::Mat &x_in, cv::Mat &P_in, cv::Mat &F_in,
		cv::Mat &H_in, cv::Mat &R_in, cv::Mat &Q_in);

	/*
	* Predicts the state and the state covariance
	* using the process model
	*/
	void Predict();

	/**
	* Updates the state by using standard Kalman Filter equations
	* z - the measurement at k+1
	*/
	void Update(const cv::Mat &z);
};


class ExtendedKalmanFilter
{
public:
	ExtendedKalmanFilter();
	~ExtendedKalmanFilter() {};

	void ProcessData(const LaserData &data_pack);

	KalmanFilter filter = KalmanFilter();

private:
	bool is_initialized;

	long long prev_timestamp;

	float noise_ax;
	float noise_ay;
	cv::Mat R_laser;
	cv::Mat H_laser;
};