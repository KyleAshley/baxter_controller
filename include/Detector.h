 #include "opencv2/objdetect/objdetect.hpp"
 #include "opencv2/highgui/highgui.hpp"
 #include "opencv2/imgproc/imgproc.hpp"

 #include <iostream>
 #include <stdio.h>

 using namespace std;
 using namespace cv;

 class Detector
 {


public:
	string window_name_;
	String cascade_name_;

	 Mat viz_;		// current image to perform detection
	 vector<Rect> detections_;
	 vector<Mat> detection_imgs_;
	 CascadeClassifier cascade_;

	 Detector();
	Detector(string);
	~Detector();
	void detect(Mat frame, bool show);		// detection on frame based on classifier xml_name


 };