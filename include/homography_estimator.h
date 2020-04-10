#pragma once
#include "estimator.h"


class HomographyEstimator : public Estimator {
public:
	HomographyEstimator();
	bool checkMatrix(cv::Mat M) override;

	int findSquare(cv::Mat &image, std::vector<std::vector<cv::Point> >& squares);
	void filterSquare(std::vector<std::vector<cv::Point> >& squares);
	void drawSquares(cv::Mat& image, std::vector<std::vector<cv::Point> >& squares);
	int IterativeSquareEstimation(cv::Mat &master, cv::Mat &sample);

	int IterativeRectEstimation(cv::Mat &image, cv::Size boardSize, int nStep);
	void iterativeBoardsEstimation(cv::Size boardSize, std::vector<cv::Mat> &boards);
	int referenceBoardMatching(cv::Size boardSize, std::string masterFilename, std::string sampleFilename);
};

