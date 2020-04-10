#include "config.h"

#include "affine_estimator.h"
#include "camera_estimator.h"
#include "homography_estimator.h"

#include <direct.h>
#include <filesystem>

using namespace cv;
using namespace std;
namespace fs = filesystem;

cv::String keys =
	"{help h usage ?   |        | show help message}"      // optional, show help optional
	"{@image_1		   | <none> | acquired board path}"         // input image is the first argument (positional)
	"{width        | 9      | chessboard width }"
	"{height       | 6      | chessboard height }"
	"{@image_2		   |<none>        | reference board path}";         // input image is the first argument (positional)


int main(int argc, char** argv) 
{

	// Initialize command line parser
	cv::CommandLineParser parser(argc, argv, keys);
	if (parser.has("help")) {
		parser.printMessage();
		return 0;
	}

	String input_image_path = parser.get<String>(0); // read @image (mandatory, error if not present)
	
	if (!parser.has("width") == 0)
		CV_Error(Error::StsError, "You must specify Board width");
	
	if (!parser.has("height") == 0)
		CV_Error(Error::StsError, "You must specify Board height");

	// NB patternSize.width = squares_columns - 1; patternSize.height = squares_rows -1;
	Size patternSize(parser.get<int>("width"), parser.get<int>("height"));

	if (!parser.check()) {
		parser.printErrors();
		return -1;
	}

	// Check Debug Dir
	if (!fs::exists(debug_folder))
		mkdir(debug_folder.c_str());

	/*"D:/Datasets/TopScan/Calibration/Epson V800/600dpi 28x20 10mm/img153.bmp"*/
	Mat sampleImg = imread(input_image_path);
	if (sampleImg.empty()) {
		cerr << "cannot load " << input_image_path << endl;
		return -1;
	}

	HomographyEstimator est;
	est.IterativeRectEstimation(sampleImg, patternSize, 10);

}