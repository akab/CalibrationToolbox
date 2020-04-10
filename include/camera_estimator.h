#pragma once
#include "config.h"
#include "homography_estimator.h"


class CameraEstimator : public HomographyEstimator {
public:
	CameraEstimator();
	bool checkMatrix(cv::Mat M) override;

	void calibration(cv::Size boardSize, std::vector<cv::Mat> &boards);
	double checkerboardCalibration(cv::Size boardSize, std::vector<cv::Mat> &boards, cv::Mat_<double> &camMat, cv::Mat_<double> &coeff);
	
};
