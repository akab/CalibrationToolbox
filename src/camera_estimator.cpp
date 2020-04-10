#include "camera_estimator.h"
#include "board_detector.h"

using namespace cv;
using namespace std;
using namespace SeeoneCalib;

CameraEstimator::CameraEstimator() : HomographyEstimator()
{
}

void CameraEstimator::calibration(Size boardSize, vector<Mat> &boards) {
	Mat_<double> camMat_est(3, 3);
	camMat_est << 519195, 0, 2550,
		0, 507085, 3509.5,
		0, 0, 1;

	Mat_<double> distCoeffs_est(1, 8);
	// "k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"
	distCoeffs_est << -19.2968,
		-0.0173796,
		-0.0131426,
		-0.0443067,
		-7.03E-07,
		15.3852,
		0.0173715,
		7.02E-07;

	double rep_err = checkerboardCalibration(boardSize, boards, camMat_est, distCoeffs_est);

	cout << endl << "Average Reprojection error: " << rep_err / boards.size() / boardSize.area() << endl;
	cout << "==================================" << endl;
	cout << "Estimated camera matrix:\n" << (Mat_<double>&)camMat_est << endl;
	cout << "Estimated distCoeffs:\n" << (Mat_<double>&)distCoeffs_est << endl;

	write(camMat_est, "camera_extrinsic.csv");
	write(distCoeffs_est, "camera_intrisic.csv");
}

double CameraEstimator::checkerboardCalibration(Size boardSize, vector<Mat> &boards, Mat_<double> &camMat, Mat_<double> &coeff) {
	// Initial points
	size_t brds_num = boards.size();
	vector< vector<Point3f> > objectPoints;
	vector< Mat > imagePoints;
	Mat corners;
	vector<Point3f> chessboard3D;
	for (int j = 0; j < boardSize.height; ++j)
		for (int i = 0; i < boardSize.width; ++i)
			chessboard3D.push_back(Point3i(i, j, 0));

	cout << endl << "Finding chessboards' corners..." << endl;
	for (size_t i = 0; i < brds_num; ++i)
	{
		cout << "processing image " << i + 1 << "... " << endl;
		bool found = findChessboardCorners(boards[i], boardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
		if (found)
		{
			// Get Subpixel accuracy on found corners
			cornerSubPix(
				boards[i], // Input image
				corners, // Initial guesses, also output
				cv::Size(11, 11), // Search window size
				cv::Size(-1, -1), // Zero zone (in this case, don't use)
				cv::TermCriteria(
					cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
					100, 0.001)
			);


			imagePoints.push_back(corners);
			objectPoints.push_back(chessboard3D);
			cout << "found " << endl;
		}
		else
			cout << "not-found " << endl;

		Mat boardWithCorners(boards[i].size(), CV_8UC3);
		cvtColor(boards[i], boardWithCorners, CV_GRAY2BGR);
		drawChessboardCorners(boardWithCorners, boardSize, corners, found);
		imwrite(debug_folder + "found_corners_" + to_string(i + 1) + ".bmp", boardWithCorners);
	}
	cout << "Done" << endl;


	// Extrinsic params
	Size imgSize = boards[0].size();
	vector<Mat> rvecs, tvecs;

	cout << "Calibrating...";
	double rep_err = calibrateCamera(
		objectPoints,
		imagePoints,
		imgSize,
		camMat,
		coeff,
		rvecs,
		tvecs,
		cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_FIX_ASPECT_RATIO |
		cv::CALIB_RATIONAL_MODEL | cv::CALIB_USE_INTRINSIC_GUESS,
		cv::TermCriteria(
			cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
			100000, 0.00001)
	);
	cout << "Done" << endl;

	return rep_err;

	return -1;
}

bool CameraEstimator::checkMatrix(cv::Mat M)
{
	return true;
}
