#pragma once
#include "estimator.h"


class AffineEstimator : public Estimator {
public:
	AffineEstimator();
	bool checkMatrix(cv::Mat M) override;

	int IterativeRectEstimation(cv::Mat &image, cv::Size boardSize, int nStep);
};