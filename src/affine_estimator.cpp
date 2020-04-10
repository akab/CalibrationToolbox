#include "affine_estimator.h"
#include "board_detector.h"

using namespace cv;
using namespace std;
using namespace SeeoneCalib;


AffineEstimator::AffineEstimator() : Estimator(TransfType::Affine)
{
}

int AffineEstimator::IterativeRectEstimation(Mat & image, Size boardSize, int nStep)
{
	vector<Mat> homographies;

	for (int i = 0; i < nStep; ++i) {
		// Find corners in image
		BoardDetector detector(BoardType::Checkerboard);
		vector<Point2f> corners;
		if (detector.findPoints(image, boardSize, &corners)) {
			Mat boardWithCorners = image.clone();
			drawChessboardCorners(boardWithCorners, boardSize, corners, true);
			//imwrite(debug_folder + "found_corners.jpg", boardWithCorners);
		}
		else {
			cerr << "cannot find board points!" << endl;
			return -1;
		}

		// Find Image Rectangle
		vector<Point2f> rectangleCorners = { corners[0], corners[boardSize.width - 1],
											corners[boardSize.width * boardSize.height - boardSize.width],
											corners[boardSize.width * boardSize.height - 1] };
		vector<Point2f> polygon;
		approxPolyDP(Mat(rectangleCorners), polygon, 5, true);
		if (polygon.size() != 4) {
			cerr << "cannot find an approximation polygonal btw corners!";
			return -1;
		}

		Mat imageRect = image.clone();
		line(imageRect, polygon[0], polygon[1], Scalar(0, 0, 255), 3, CV_AA, 0);
		line(imageRect, polygon[1], polygon[3], Scalar(0, 0, 255), 3, CV_AA, 0);
		line(imageRect, polygon[3], polygon[2], Scalar(0, 0, 255), 3, CV_AA, 0);
		line(imageRect, polygon[2], polygon[0], Scalar(0, 0, 255), 3, CV_AA, 0);
		//imwrite(debug_folder + "image_rect.jpg", imageRect);

		// Compute Correct Rectangle
		Rect boundRect = boundingRect(rectangleCorners);
		vector<Point2f> rect = { Point2f(boundRect.x, boundRect.y),
								Point2f(boundRect.x + boundRect.width, boundRect.y),
								Point2f(boundRect.x, boundRect.y + boundRect.height),
								 };

		Mat imageRightRect = image.clone();
		rectangle(imageRightRect, boundRect, Scalar(0, 255, 0), 3, 8, 0);
		//imwrite(debug_folder + "correct_rect.jpg", imageRightRect);

		// Find H
		vector<Point2f> poly_pts = { Point2f(polygon[0].x, polygon[0].y), Point2f(polygon[1].x, polygon[1].y),
								Point2f(polygon[2].x, polygon[2].y) };

		Mat H = getAffineTransform(poly_pts, rect);

		// Show results
		Mat imageDiff = image.clone();
		line(imageDiff, polygon[0], polygon[1], Scalar(0, 0, 255), 5, CV_AA, 0);
		line(imageDiff, polygon[1], polygon[3], Scalar(0, 0, 255), 5, CV_AA, 0);
		line(imageDiff, polygon[3], polygon[2], Scalar(0, 0, 255), 5, CV_AA, 0);
		line(imageDiff, polygon[2], polygon[0], Scalar(0, 0, 255), 5, CV_AA, 0);
		rectangle(imageDiff, boundRect, Scalar(0, 255, 0), 5);
		//imwrite(debug_folder + "difference_before" + to_string(i + 1) + ".bmp", imageDiff);

		Mat transfImage = image.clone();
		warpAffine(image, transfImage, H, image.size(), INTER_CUBIC, BORDER_REFLECT);
		rectangle(transfImage, boundRect, Scalar(0, 255, 0), 5);
		//imwrite(debug_folder + "difference_after" + to_string(i + 1) + ".bmp", transfImage);

		cout << "==================================" << endl;
		cout << "Estimated Homography matrix:\n" << (Mat_<double>&)H << endl;

		// Homography check
		checkMatrix(H);
		homographies.push_back(H);

		// Write estimated homography
		//write(H, debug_folder + "estimated_homography" + to_string(i + 1) + ".csv");

		// Update starting image
		Mat *rectified = new Mat(image.size(), image.type());
		warpAffine(image, *rectified, H, image.size(), INTER_CUBIC, BORDER_REFLECT);

		image = *rectified;

		imwrite(debug_folder + "rectified_" + to_string(i + 1) + ".bmp", image);
	}

	Mat aff = Mat::ones(homographies[0].size(), homographies[0].type());
	for (auto a : homographies) {
		aff = aff.mul(a);
		checkMatrix(aff);
	}

	T = aff;
	write(T, debug_folder + "composite_aff.csv");

	return 0;
}

bool AffineEstimator::checkMatrix(Mat M)
{
	// Check if matrix is close to singular
	Mat S, V, D;
	SVDecomp(M, S, V, D, CV_HAL_SVD_FULL_UV);
	double conditionNumber = S.at<double>(0, 0) / S.at<double>(2, 0);
	cout << S << endl;
	if (conditionNumber < pow(10, 7))
		return true;
	else {
		cout << "...SVD check failed " << endl;
		return false;
	}
}
