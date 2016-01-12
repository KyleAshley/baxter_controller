/*
************************************************************************************
*	PointCloudCapture.cpp
*	Author: Kyle Ashley
*	
*	Description: Publishes Kinect raw image data to rostopic /kinect/raw_image
************************************************************************************
*/

#include <stdio.h>
#include <stdlib.h>
#include <cmath>

#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>


#include "PointCloudCapture/PointCloudCapture.h"

using namespace std;
 
int main(int argc, char **argv)
{
 
	ros::init(argc, argv, "kinect_raw_img_pub");
	 
	ros::NodeHandle n;
	//image_transport::ImageTransport it;

	image_transport::ImageTransport it(n);
    image_transport::Publisher pub_raw = it.advertise("kinect/rgb_image", 1);
    ros::Publisher pub_pointCloud = n.advertise<sensor_msgs::PointCloud2> ("kinect/cloud", 1);
    
    ros::Rate loop_rate(30);
    //image_transport::Publisher pub_depth = it.advertise("kinect/depth_img", 1);
 	PointCloudCapture* cam = new PointCloudCapture();
 	cam->startCapture();

 	ROS_INFO("Publishing Kinect point cloud on: kinect/cloud");
 	ROS_INFO("Publishing Kinect rgb image on:   kinect/rgb_image");

	while (ros::ok())
	{
		// capture image from kinect
	    InputData src;
	    cam->getFrame(src);

		/*// make the point cloud
		sensor_msgs::PointCloud2 pc_msg;
		pcl::toROSMsg(*src.srcCloud, pc_msg);

		// publish the cloud
		pub_pointCloud.publish(pc_msg);
		*/
		
		// make the rgb image
		cv_bridge::CvImage cv_image;
		cv_image.image = src.srcImg;
		cv_image.encoding = "bgr8";

		// make the ROS message for the rgb image
		sensor_msgs::Image ros_image;
		cv_image.toImageMsg(ros_image);

		// publish the rgb image
		pub_raw.publish(ros_image);
		cv::imshow("Published", cv_image.image);
		cv::waitKey(5);
		
		
		ros::spinOnce();
		loop_rate.sleep();
	}
 
} 