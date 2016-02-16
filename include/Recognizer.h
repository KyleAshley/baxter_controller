#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "boost/filesystem.hpp"   // includes all needed Boost.Filesystem declarations
#include "Detector.h"
#include "settings.h"

#include <iostream>
#include <fstream>
#include <sstream>

using namespace cv;
using namespace std;

class Recognizer
{



public:
	String profile_directory_;

	Mat viz_;						//visualization of recognition
	Detector detector_;
	
    vector<Mat> images_;			// training images
    vector<int> labels_;			// training labels (profile ID #)
    int num_profiles_;

    vector<int> prediction_ids_;
    double confidence_;

	Ptr<FaceRecognizer> model_;		// predictive model for reognizer

	Recognizer();
	~Recognizer();
	void update();				// reads training images from directory profile_directory_
	Mat readProfileImg(int, int);
	void recognizeDetections(Mat, bool);		// recognizes faces in a Detector object

};