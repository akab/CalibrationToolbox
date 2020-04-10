#pragma once
#include "config.h"

class Estimator {
public:
	Estimator(SeeoneCalib::TransfType type);

	cv::Mat getTransformation();
	virtual bool checkMatrix(cv::Mat M) = 0;

protected:
	cv::Mat T;

	void write(cv::Mat M, std::string filename);
};

template<class T> std::ostream& operator<<(std::ostream& out, const cv::Mat_<T>& mat)
{
	for (int j = 0; j < mat.rows; ++j)
		for (int i = 0; i < mat.cols; ++i)
			out << mat(j, i) << " ";

	return out;
}

static inline double eucl_dist(double x, double xp, double y, double yp) {
	return (sqrt((x - xp) * (x - xp) + (y - yp) * (y - yp)));
}
