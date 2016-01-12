#pragma once

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64.h"

#include <stdio.h>    
#include <opencv2/opencv.hpp>    
#include <opencv2/stitching/stitcher.hpp>  

#include "util.h"


using namespace std;
using namespace cv;



class gestureTracker
{
public:
	ros::NodeHandle n;
	image_transport::ImageTransport it;

	ros::Subscriber l_hand_sub;
	ros::Subscriber r_hand_sub;
	image_transport::Subscriber depth_img_sub;

	ros::Publisher r_hand_state_pub;
	ros::Publisher l_hand_state_pub;


	Point l_hand_pos, r_hand_pos;
	Point l_hand_pos_prev, r_hand_pos_prev;
	Mat l_hand_roi, r_hand_roi; 
	int r_hand_gesture_id, l_hand_gesture_id;

	Mat depth_img;
	bool has_depth_data;

	slidingWindow l_roi_size_filter;
	slidingWindow r_roi_size_filter;

	slidingWindow r_hand_palm_x_filter;
	slidingWindow r_hand_palm_y_filter;
	slidingWindow l_hand_palm_x_filter;
	slidingWindow l_hand_palm_y_filter;

	slidingWindow l_hand_state_filter;
	slidingWindow r_hand_state_filter;

	bool in_use;
	
	
	gestureTracker(ros::NodeHandle n);
	~gestureTracker();
	void l_hand_cb(const std_msgs::Int32MultiArray::ConstPtr& array);
	void r_hand_cb(const std_msgs::Int32MultiArray::ConstPtr& array);
	void depth_img_cb(const sensor_msgs::ImageConstPtr& msg_ptr);

	void detectGesture(Mat hand_roi, string window_name);

	
private:
	

};