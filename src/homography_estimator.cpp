#include "homography_estimator.h"
#include "board_detector.h"

using namespace cv;
using namespace std;
using namespace SeeoneCalib;


HomographyEstimator::HomographyEstimator() : Estimator(TransfType::Homography)
{
}

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
static double angle(Point pt1, Point pt2, Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1 * dy2) / sqrt((dx1*dx1 + dy1 * dy1)*(dx2*dx2 + dy2 * dy2) + 1e-10);
}

int HomographyEstimator::findSquare(Mat & image, vector<vector<Point> >& squares)
{
	squares.clear();

	Mat pyr, timg, gray0(image.size(), CV_8U), gray;

	// Down-scale and upscale the image to filter out the noise
	pyrDown(image, pyr, Size(image.cols / 2, image.rows / 2));
	pyrUp(pyr, timg, image.size());
	vector<vector<Point> > contours;

	// find squares in every color plane of the image
	const int N = 11;
	const int thresh = 50;
	for (int c = 0; c < 3; c++)
	{
		int ch[] = { c, 0 };
		mixChannels(&timg, 1, &gray0, 1, ch, 1);

		// try several threshold levels
		for (int l = 0; l < N; l++)
		{
			// hack: use Canny instead of zero threshold level.
			// Canny helps to catch squares with gradient shading
			if (l == 0)
			{
				// apply Canny. Take the upper threshold from slider
				// and set the lower to 0 (which forces edges merging)
				Canny(gray0, gray, 0, thresh, 5);
				// dilate canny output to remove potential
				// holes between edge segments
				dilate(gray, gray, Mat(), Point(-1, -1));
			}
			else
			{
				// apply threshold if l!=0:
				//     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
				gray = gray0 >= (l + 1) * 255 / N;
			}

			// find contours and store them all as a list
			findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);


			// test each contour
			vector<Point> approx;
			for (size_t i = 0; i < contours.size(); i++)
			{
				// approximate contour with accuracy proportional
			   // to the contour perimeter
				approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.02, true);

				// square contours should have 4 vertices after approximation
				// relatively large area (to filter out noisy contours)
				// and be convex.
				// Note: absolute value of an area is used because
				// area may be positive or negative - in accordance with the
				// contour orientation
				if (approx.size() == 4 &&
					fabs(contourArea(approx)) > 1000 &&
					isContourConvex(approx))
				{
					double maxCosine = 0;

					for (int j = 2; j < 5; j++)
					{
						// find the maximum cosine of the angle between joint edges
						double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
						maxCosine = MAX(maxCosine, cosine);
					}

					// if cosines of all angles are small
					// (all angles are ~90 degree) then write quandrange
					// vertices to resultant sequence
					if (maxCosine < 0.3)
						squares.push_back(approx);
				}
			}
		}
	}

	return 0;
}

void HomographyEstimator::filterSquare(vector<vector<Point>>& squares)
{
	vector<vector<Point>> filteredSquare;
	for (size_t i = 0; i < squares.size(); i++)
	{
		double d1 = eucl_dist(squares[i][0].x, squares[i][1].x, squares[i][0].y, squares[i][1].y);
		double d2 = eucl_dist(squares[i][1].x, squares[i][2].x, squares[i][1].y, squares[i][2].y);
		double d3 = eucl_dist(squares[i][2].x, squares[i][3].x, squares[i][2].y, squares[i][3].y);
		double d4 = eucl_dist(squares[i][3].x, squares[i][0].x, squares[i][3].y, squares[i][0].y);
		int eps = 10;
		if (d1 > 472.44 - eps && d1 < 472.44 + eps &&
			d2 > 472.44 - eps && d2 < 472.44 + eps &&
			d3 > 472.44 - eps && d3 < 472.44 + eps &&
			d4 > 472.44 - eps && d4 < 472.44 + eps)
		{
			filteredSquare.push_back(squares[i]);
		}
	}

	squares = filteredSquare;
}

void HomographyEstimator::drawSquares(Mat & image, vector<vector<Point>>& squares)
{
	for (size_t i = 0; i < squares.size(); i++)
	{
		Point* p = &squares[i][0];
		int n = (int) squares[i].size();
		polylines(image, &p, &n, 1, true, Scalar(0, 255, 0), 3, LINE_AA);
	}
}

int HomographyEstimator::IterativeSquareEstimation(Mat & master, Mat & sample)
{
	vector<vector<Point> > masterSquare, sampleSquare;
	
	// Find square in Master
	findSquare(master, masterSquare);
	filterSquare(masterSquare);

	Mat masterWithSq = master.clone();
	drawSquares(masterWithSq, masterSquare);
	imwrite(debug_folder + "master_squares.bmp", masterWithSq);

	// Find square in Sample
	findSquare(sample, sampleSquare);
	filterSquare(sampleSquare);

	Mat sampleWithSq = sample.clone();
	drawSquares(sampleWithSq, sampleSquare);
	imwrite(debug_folder + "sample_squares.bmp", sampleWithSq);

	// Match points
	// TODO
	//Mat h = findHomography(s, m, RANSAC, 3.f);
	//write(h, "square_estimation.csv");

	//Mat sampleWarped(sample.size(), sample.type());
	//warpPerspective(sample, sampleWarped, h, sample.size(), INTER_CUBIC, BORDER_REFLECT);
	//imwrite(debug_folder + "rectified_sample.bmp", sampleWarped);

	return 0;
}

int HomographyEstimator::IterativeRectEstimation(Mat &image, Size boardSize, int nStep)
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

		// Compute Correct Rectangle
		Rect boundRect = boundingRect(rectangleCorners);
		vector<Point2f> rect = { Point2f(boundRect.x, boundRect.y),
								Point2f(boundRect.x + boundRect.width, boundRect.y),
								Point2f(boundRect.x, boundRect.y + boundRect.height),
								Point2f(boundRect.x + boundRect.width, boundRect.y + boundRect.height) };

		rectangle(imageRect, boundRect, Scalar(0, 255, 0), 3, 8, 0);
		imwrite(debug_folder + "rectangles_" + to_string(i+1) + ".jpg", imageRect);

		// Find H
		vector<Point2f> poly_pts = { Point2f(polygon[0].x, polygon[0].y), Point2f(polygon[1].x, polygon[1].y),
									Point2f(polygon[2].x, polygon[2].y), Point2f(polygon[3].x, polygon[3].y) };

		//Mat H = getPerspectiveTransform(poly_pts, rect);
		Mat H = findHomography(poly_pts, rect, LMEDS, 0.01f);

		// Show results
		Mat imageDiff = image.clone();
		//line(imageDiff, polygon[0], polygon[1], Scalar(0, 0, 255), 5, CV_AA, 0);
		//line(imageDiff, polygon[1], polygon[3], Scalar(0, 0, 255), 5, CV_AA, 0);
		//line(imageDiff, polygon[3], polygon[2], Scalar(0, 0, 255), 5, CV_AA, 0);
		//line(imageDiff, polygon[2], polygon[0], Scalar(0, 0, 255), 5, CV_AA, 0);
		rectangle(imageDiff, boundRect, Scalar(0, 255, 0), 5);
		imwrite(debug_folder + "image_before" + to_string(i + 1) + ".bmp", imageDiff);

		Mat transfImage = image.clone();
		warpPerspective(image, transfImage, H, image.size(), INTER_CUBIC, BORDER_REFLECT);
		rectangle(transfImage, boundRect, Scalar(0, 255, 0), 5);
		imwrite(debug_folder + "image_after" + to_string(i + 1) + ".bmp", transfImage);

		cout << "==================================" << endl;
		cout << "Estimated Homography matrix:\n" << (Mat_<double>&)H << endl;

		// Homography check
		checkMatrix(H);
		homographies.push_back(H);

		// Write estimated homography
		write(H, debug_folder + "estimated_homography" + to_string(i + 1) + ".csv");

		// Update starting image
		Mat *rectified = new Mat(image.size(), image.type());
		warpPerspective(image, *rectified, H, image.size(), INTER_CUBIC, BORDER_REFLECT);

		image = *rectified;

		imwrite(debug_folder + "rectified_" + to_string(i+1) + ".bmp", image);
	}

	Mat homo = Mat::ones(homographies[0].size(), homographies[0].type());
	for (auto a : homographies) {
		homo = homo.mul(a);
		checkMatrix(homo);
	}

	T = homo;
	write(T, debug_folder + "composite_homo.csv");
	
	return 0;
}

void HomographyEstimator::iterativeBoardsEstimation(Size boardSize, vector<Mat> &boards) {
	Mat_<double> H(3, 3);
	H << 1.00607, 0.00157, -11.0, 0.00539, 1.00607, -27.5, 0, 0, 1;
	// Initial points
	size_t brds_num = boards.size();
	vector< vector<Point3f> > objectPoints;
	vector<Point2f> corners;
	vector< vector<Point2f> > imagePoints;

	cout << endl << "Finding chessboards' corners..." << endl;
	for (size_t i = 0; i < brds_num; ++i)
	{
		cout << "processing image " << i + 1 << "... ";
		bool found = findChessboardCorners(boards[i], boardSize, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
		if (found)
		{
			// Get Subpixel accuracy on found corners
			cornerSubPix(
				boards[i], // Input image
				corners, // Initial guesses, also output
				Size(11, 11), // Search window size
				Size(-1, -1), // Zero zone (in this case, don't use)
				TermCriteria(
					TermCriteria::EPS | TermCriteria::COUNT,
					100, 0.001)
			);


			imagePoints.push_back(corners);
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


	cout << "Estimating Homography...";

	int half;
	vector<Point2f> src, dst;
	if (imagePoints.size() % 2 == 0)
		half = imagePoints.size() / 2;
	else
		half = imagePoints.size() / 2 + 1;

	for (int i = 0; i < imagePoints.size(); ++i) {
		if (i < half) {
			for (auto pt : imagePoints[i])
				src.push_back(pt);
		}
		else {
			for (auto pt : imagePoints[i])
				dst.push_back(pt);
		}
	}

	H = findHomography(src, dst, LMEDS, 3.0f);

	cout << "==================================" << endl;
	cout << "Estimated Homography matrix:\n" << (Mat_<double>&)H << endl;

	// Check if file exists
	T = H;
	write(T, debug_folder + "estimated_homography.csv");


	cout << "Done" << endl;
}

int HomographyEstimator::referenceBoardMatching(Size boardSize, string masterFilename, string sampleFilename)
{
	Mat master = imread(masterFilename);
	Mat sample = imread(sampleFilename);

	/***************** Find Corners *****************/
	vector<Point2f> master_corners, sample_corners;
	
	BoardDetector detector(BoardType::Checkerboard);
	detector.findPoints(master, boardSize, &master_corners);
	detector.findPoints(sample, boardSize, &sample_corners);

	if (master_corners.size() != sample_corners.size()) {
		cerr << "number of corners btw master/sample not matching" << endl;
		return -1;
	}

	Mat masterWithCorners = master.clone();
	drawChessboardCorners(masterWithCorners, boardSize, master_corners, true);
	imwrite(debug_folder + "masterCorners.jpg", masterWithCorners);

	Mat sampleWithCorners = sample.clone();
	drawChessboardCorners(sampleWithCorners, boardSize, sample_corners, true);
	imwrite(debug_folder + "sampleCorners.jpg", sampleWithCorners);

	Mat sampleRect;

	/***************** Compute H *****************/
	Mat h = findHomography(master_corners, sample_corners, RANSAC, 3.f);
	checkMatrix(h);
	cout << "==================================" << endl;
	cout << "Estimated Homography matrix:\n" << (Mat_<double>&)h << endl;

	T = h;
	write(T, debug_folder + "estimated_homography.csv");

	/***************** Correct Image *****************/
	Mat sampleRectified;
	warpPerspective(sample, sampleRectified, h, master.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar(255, 255, 255));
	imwrite(debug_folder + "sampleRectified.jpg", sampleRectified);

	return 0;
}

bool HomographyEstimator::checkMatrix(Mat M)
{
	if (T.empty())
		return false;

	bool c1, c2, c3;
	// 1) Check that det(T) > 0
	if (determinant(T) > 0)
		c1 = true;
	else {
		cout << "...determinant check failed " << endl;
		c1 = false;
	}

	// 2) Compute SVD and check that ratio of the first-to-last singular value (condition number) is not too high (close to singular, order of 1.0E7).
	Mat S, V, D;
	SVDecomp(T, S, V, D, CV_HAL_SVD_NO_UV);
	double conditionNumber = S.at<double>(0, 0) / S.at<double>(2, 0);
	if (conditionNumber < pow(10, 7))
		c2 = true;
	else {
		cout << "...SVD check failed " << endl;
		c2 = false;
	}

	// 3) Check perspectivity (High perspectivity -> distortions -> error)
	double a = T.at<double>(0, 0);
	double b = T.at<double>(0, 1);
	double d = T.at<double>(1, 0);
	double e = T.at<double>(1, 1);
	double q = (a*d + b * e) / (a*e - b * d);
	if (abs(q) <= 0.01)
		c3 = true;
	else {
		cout << "...perspectivity check failed " << endl;
		c3 = false;
	}

	bool res = c1 * c2 * c3;
	res == 0 ?
		cout << "is homography good? false" << endl : cout << "is homography good? true" << endl;

	return res;
}
