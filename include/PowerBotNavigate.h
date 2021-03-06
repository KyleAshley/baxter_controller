/*
* Author: Kyle Ashley, Andoni Aguirrezabal
*
* Description: ROS node that subscribes to results of a detection algorithm 
*              given by a bounding rectangle. Powerbot navigation
*              executes path planning to location and publishes status.
*
*              To use without detection (only x,y coordinates) publish an 
*              artifically generated rectangle around the desired location
*
*/

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/String.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

#include "visualization_msgs/Marker.h"

//#include <ar_track_alvar/AlvarMarker.h>
//#include <ar_track_alvar/AlvarMarkers.h>

#include "tf/transform_listener.h"
#include "tf/tf.h"

#include <cmath>
#include <string>
#include <deque>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "PointCloudCapture/PointCloudCapture.h"
#include "PowerBotClient.h"
#include "Recognizer.h"
#include "settings.h"
#include <stdio.h>
#include <typeinfo>

bool isPosOutlier(vector<pcl::PointXYZ> prev_pos, pcl::PointXYZ point);

class PowerBotNavigate
{


public:
    tf::TransformListener listener;

    string ar_person_topic;
    int ar_person_id;
    std_msgs::Int32MultiArray person_pose;

	ros::NodeHandle n_;
    ros::Subscriber sub_face_navigate_;
    ros::Subscriber sub_location_navigate_;
    ros::Subscriber sub_ar_navigate_;
    ros::Subscriber sub_rotate_;
    ros::Subscriber sub_ar_marker_;             // transforms AR tag coordinates at 10Hz



    image_transport::Publisher pub_raw_;        // publish kinect data used in face detection
    ros::Publisher pub_personReached_;          // boolean topic, 1 = successfull navigation to person
    ros::Publisher pub_locationReached_;        // boolean topic, 1 = successfull navigation to location
    ros::Publisher pub_rotationReached_;        // boolean topic, 1 = successfull navigation to location
    ros::Publisher pub_arTagReached_;        // boolean topic, 1 = successfull navigation to location

    ros::Publisher pub_arToLocInternal_;        // internal topic to publish to navigate to AR tag location

    PointCloudCapture* pcc_;                    // pointCoudCapture for front facing kinect
    Recognizer recognizer_;                     // face recognizer class

    int imgWidth_;				                // dimensions of kinect data
    int imgHeight_;
    InputData kin_data_;				        // kinect data

    pcl::PointXYZ pos_;			                // destination pose in world frame
    pcl::PointXYZ curr_target_;                 // raw destination data from ROS msgs
    float target_theta_;                        // desired orientation at target location

    // records of previous navigation locations
    std::deque<pcl::PointXYZ> waypoints_;
    pcl::PointXYZ prev_pos_;

    PowerBotClient pbClient_;
    bool destination_reached_;          // internal flag for completed navigation

    PowerBotNavigate();
    ~PowerBotNavigate();

    // call to command powerbot to move to a desired location contained in ROI bounded by tl, and br points
    void navigateToDetectionCb(const std_msgs::String::ConstPtr& msg);
    void navigateToLocationCb(const std_msgs::Int32MultiArray::ConstPtr& coords);
    void navigateToARTagCb(const std_msgs::String::ConstPtr& msg);
    bool navigateToARLocation(bool initial_navigation);

    void rotationCb(const std_msgs::Int32MultiArray::ConstPtr& coords);
    void arMarkerCallback(const ros::TimerEvent& event);

    bool navigateToROI(cv::Point, cv::Point, bool initial_navigation);
    Eigen::Vector4f getCenter(int centerX, int centerY, int imgWidth, int imgHeight,
                          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr srcCloud);

};