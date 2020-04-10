#include "..\include\estimator.h"
#include <iostream>

using namespace cv;
using namespace std;

Estimator::Estimator(SeeoneCalib::TransfType type)
{
	switch (type)
	{
	case SeeoneCalib::Affine:
		T = Mat(2, 3, CV_32FC1);
		break;
	case SeeoneCalib::Homography:
		T = Mat(3, 3, CV_32FC1);
		break;
	default:
		break;
	}

}

void Estimator::write(cv::Mat M, std::string filename)
{
	// Check if file exists
	struct stat buf;
	if (stat(filename.c_str(), &buf) != -1)
		remove(filename.c_str());

	ofstream outdata;
	outdata.open(filename, ios::app);

	for (int i = 0; i < M.rows; ++i) {
		for (int j = 0; j < M.cols; ++j) {
			outdata << M.at<double>(i, j) << ";";
			if (j % 2 == 0 && j != 0)
				outdata << endl;
		}
	}

	outdata.close();
}

cv::Mat Estimator::getTransformation()
{
	return T;
}
