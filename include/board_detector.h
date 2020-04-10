#pragma once
#include "config.h"


class BoardDetector {
public:
	BoardDetector(SeeoneCalib::BoardType _type);

	bool findPoints(cv::Mat &image, cv::Size boardSize, std::vector<cv::Point2f> *foundCorners);

private:
	SeeoneCalib::BoardType type;

	bool findChessCorners(cv::Mat &image, cv::Size boardSize, std::vector<cv::Point2f> *foundCorners);
};