#include "board_detector.h"

using namespace cv;
using namespace std;
using namespace SeeoneCalib;

BoardDetector::BoardDetector(BoardType _type) : type(_type)
{
}

bool BoardDetector::findPoints(cv::Mat & image, cv::Size boardSize, std::vector<cv::Point2f>* foundCorners)
{
	switch (type)
	{
		case SeeoneCalib::Checkerboard:
			return findChessCorners(image, boardSize, foundCorners);

		case SeeoneCalib::ChArUco:
			break;

		case SeeoneCalib::Circles:
			break;

		case SeeoneCalib::AsymmetricCircles:
			break;

		default:
			break;
	}

	return false;
}

bool BoardDetector::findChessCorners(cv::Mat & image, cv::Size boardSize, std::vector<cv::Point2f>* foundCorners)
{
	Mat sample_thr(image.size(), CV_8UC1);
	if (image.channels() == 3) {
		cvtColor(image, sample_thr, CV_BGR2GRAY);
		threshold(sample_thr, sample_thr, 125, 255, CV_THRESH_BINARY);
		imwrite(debug_folder + "tresholded.jpg", sample_thr);
	}

	bool found = findChessboardCorners(sample_thr, boardSize, *foundCorners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
	if (found)
		cornerSubPix(
			sample_thr, // Input image
			*foundCorners, // Initial guesses, also output
			cv::Size(11, 11), // Search window size
			cv::Size(-1, -1), // Zero zone (in this case, don't use)
			cv::TermCriteria(
				cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
				100, 0.001)
		);
	else 
		return false;

	return true;
}
