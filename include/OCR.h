#include <ros/ros.h>
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/String.h"

#include <cmath>
#include <string>
#include <deque>
#include <time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

#define VERBOSE true

using namespace cv;
using namespace std;


#define PATH "/home/kyle/catkin_ws/src/baxter_sp/src/text_detection"
#define VERBOSE true

class OCR
{


public:
	int k1, k2;
	cv::Point center;

	ros::NodeHandle n;
	image_transport::ImageTransport it;
	ros::Subscriber sub_bookTitle;
	image_transport::Subscriber sub_rh_image;
	image_transport::Subscriber sub_lh_image;
    ros::Publisher pub_bookLocation;  

	cv::Mat img;
	cv::Mat rh_img;
	cv::Mat lh_img;
	cv::Mat results;

	std::vector<cv::Mat> textROIs;				// images of detected text regions
	std::vector<cv::Rect> textRects;			// rectangle bounding boxes of detected ROIs

	std::vector<std::string> textStrings;

	std::vector<std::string> desired_titles;
	string detected_match;

	OCR(ros::NodeHandle n);
	~OCR();

	void readText();
	int LongestCommonSubstring(const string& str1, const string& str2);
	void detectLetters();
	void findMatch();
	void bookTitleCb(const std_msgs::String::ConstPtr& msg);
	void rhImageCb(const sensor_msgs::ImageConstPtr& msg);
	void lhImageCb(const sensor_msgs::ImageConstPtr& msg);
};