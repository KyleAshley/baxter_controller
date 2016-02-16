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


#include "PowerBotNavigate.h"

using namespace std;

/*
bool isPosOutlier(vector<pcl::PointXYZ> prev_pos, pcl::PointXYZ point)
{
    double sumx, sumz, avgx, avgz;
    double xthresh = 0.1;
    double zthresh = xthresh;

    for(int i = 0; i < prev_pos.size(); i++)
    {
        sumx += prev_pos[i].x;
        sumz += prev_pos[i].z;
    }

    avgx = sumx / prev_pos.size();
    avgz = sumz / prev_pos.size();

    if(point.x < avgx + xthresh && point.x > avgx - xthresh)
        if(point.z < avgz + zthresh && point.z > avgz - zthresh)
            return true;
        else
            return false;
    else
        return false;
}

*/

PowerBotNavigate::~PowerBotNavigate(){};


Eigen::Vector4f PowerBotNavigate::getCenter(int centerX, int centerY, int imgWidth, int imgHeight,
                          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr srcCloud) {
    int pixelWidth = 10;
    int minX = 0, maxX = 0, minY = 0, maxY = 0;
    
    pcl::PointXYZRGBA tempPos;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
    
    Eigen::Vector4f centroid;
    
    //minX
    if((centerX - pixelWidth) <= 0) {
        minX = 0;
    } else {
        minX = centerX - pixelWidth;
    }
    
    //maxX
    if((centerX + pixelWidth) >= imgWidth) {
        maxX = imgWidth;
    } else {
        maxX = centerX + pixelWidth;
    }
    
    //minY
    if((centerY - pixelWidth) <= 0) {
        minY = 0;
    } else {
        minY = centerY - pixelWidth;
    }
    
    //maxY
    if((centerY + pixelWidth) >= imgHeight) {
        maxY = imgHeight;
    } else {
        maxY = centerY + pixelWidth;
    }
    
//    cout << "Center {X, Y}: {" << centerX << ", " << centerY << "}\n";
//    cout << "{MinX, MaxX}: {" << minX << ", " << maxX << "}\n";
//    cout << "{MinY, MaxY}: {" << minY << ", " << maxY << "}\n";
    
    for(int i = minX; i <= maxX; i++) {
        for(int j = minY; j <= maxY; j++) {
            tempPos = srcCloud->points[j * srcCloud->width + i];
//            cout << "Checking: {" << tempPos.x << ", " << tempPos.y << ", "
//                     << tempPos.z << "}\n";
            if (std::isnan(tempPos.x) || std::isnan(tempPos.y)) {
                //PASS
            } else {
//                cout << "Adding: {" << tempPos.x << ", " << tempPos.y << ", "
//                     << tempPos.z << "}\n";
                tempCloud->points.push_back(tempPos);
            }
        }
    }
//    cout << "Full PC Size: " << srcCloud->size() << "\n";
//    cout << "Filtered PC Size: " << tempCloud->size() << "\n";
    pcl::compute3DCentroid(*tempCloud, centroid);
    return centroid;
}

bool PowerBotNavigate::navigateToROI(cv::Point roi_tl, cv::Point roi_br, bool initial_navigation)
{
    cv::Point detectionCenter;

    if(pbClient_.getRunningWithLock())
    {
        pbClient_.requestUpdate();
        //pcc_->getFrame(kin_data_);

        // get (x,y) of centroid of rectangular detection
        detectionCenter.x = (roi_tl.x + roi_br.x)/2.0;
        detectionCenter.y = (roi_tl.y + roi_br.y)/2.0;

        rectangle(kin_data_.srcImg, roi_tl, roi_br,  cv::Scalar(255, 0, 0), 2);
        circle(kin_data_.srcImg, detectionCenter, 3, cv::Scalar(255, 0, 0), 2);
        imshow("center", kin_data_.srcImg);

        // transform to world frame
        pos_.getArray4fMap() = this->getCenter(detectionCenter.x, detectionCenter.y, imgWidth_, imgHeight_, kin_data_.srcCloud);

        ROS_INFO("Image Center: { %d, %d }", detectionCenter.x, detectionCenter.y);
        ROS_INFO("Marker Center (WORLD) {X,Y,Z} = { %f, %f, %f }", pos_.x, pos_.y, pos_.z);

        // make the desired coordinates shifted slightly to avoid collision?

        pos_.x = (pos_.x * (1000));
        pos_.z = (pos_.z * 1000) - 500;
        if(pos_.z < 0) { pos_.z = 0; }
        
        if((abs(pos_.x) >= 2500) || (abs(pos_.z) >= 3500) ||
            std::isnan(pos_.x) || std::isnan(pos_.z)) {
            return false;

        } else {

            
            // request update
            pbClient_.requestUpdate();
            ROS_INFO("Current Location: {X,Y} = { %f, %f }", pbClient_.pbX, pbClient_.pbY);
            
            // transform destination into world frame
            pbClient_.transformPoints(pos_.x, pos_.z);
            ROS_INFO("Desired PowerBot Coordinates {X,Y} = { %f, %f }", pos_.x, pos_.z);

            // add first waypoint and begin navigation
            if(initial_navigation)
            {
                ROS_INFO("Begining Initial Navigation");
                waypoints_.push_back(pos_);
                pbClient_.moveTo(waypoints_.front().x, waypoints_.front().z);
                //usleep(5000000);
            }

            // if latest data deviates enough from current navigation, add waypoint
            if((pos_.x >= waypoints_.front().x + 100 || pos_.x <= waypoints_.front().x - 100) || (pos_.z >= waypoints_.front().z + 100 || pos_.z <= waypoints_.front().z - 100))
            {
                 ROS_INFO("Adding Waypoint");
                waypoints_.push_back(pos_);
            }

            pbClient_.requestUpdate();

            // if front WP is reached or PB stopped
            if(((pbClient_.pbX <= waypoints_.front().x + 100 && pbClient_.pbX >= waypoints_.front().x - 100) && (pbClient_.pbY <= waypoints_.front().z + 100 && pbClient_.pbY >= waypoints_.front().z - 100)) || waypoints_.size() > 10)
            {   
                ROS_INFO("Waypoint Reached");
                ROS_INFO("Current Location: {X,Y} = { %f, %f }", pbClient_.pbX, pbClient_.pbY);
                waypoints_.pop_front();
                ROS_INFO("Waypoints Left = %f", (float)waypoints_.size());

                if(waypoints_.empty())
                    return true;
                else
                {
                    ROS_INFO("Moving to Next Waypoint: {X,Y} = { %f, %f }", waypoints_.front().x, waypoints_.front().z);
                    pbClient_.moveTo(waypoints_.back().x, waypoints_.back().z);
                    waypoints_.clear();
                    return false;
                }
            }
            else
                return false;
        }
    }

    return true;
}

void PowerBotNavigate::navigateToDetectionCb(const std_msgs::String::ConstPtr& msg)
{
    // get desired profile ID from command message
    pcc_ = new PointCloudCapture("A00367801249047A");
    pcc_->startCapture();
    pcc_->getFrame(kin_data_);

    curr_target_.x = -1.0;
    curr_target_.z = -1.0;
    curr_target_.y = -1.0;
    waypoints_.clear();

    string id_str = msg->data.c_str();
    int desired_id = atoi(id_str.c_str());
    
    stringstream ss;
    ss << "Received Command to navigate to person" << desired_id << "\n";
    string rx_msg = ss.str();
    ROS_INFO(rx_msg.c_str());

    //recognizer_.update();         // update PCA model with training data
    //ROS_INFO("Updating Recognizer...");

    destination_reached_ = false;
    std_msgs::String status_msg;

    bool initial_navigation = true;

    while(!destination_reached_)
    {
        //status_msg.data = "0";
        //pub_personReached_.publish(status_msg);
        pcc_->getFrame(kin_data_);
        //pcc_->getFrame(kin_data_.srcCloud);
        //pcc_->getPreviousCloud(kin_data_.srcCloud);

        imgWidth_ = kin_data_.srcImg.size().width;
        imgHeight_ = kin_data_.srcImg.size().height;

        // show raw img
        imshow("Kinect", kin_data_.srcImg);

        // show cloud visualizer
        waitKey(30);

        if( !kin_data_.srcImg.empty() )
        {
            recognizer_.recognizeDetections(kin_data_.srcImg, true);
        
            // iterate over recognition labels and find any matches
            if(strcmp(NAVIGATE,"face"))
            {
                for(int i = 0; i < recognizer_.prediction_ids_.size(); i++)
                {
                    ROS_INFO("Found: " + recognizer_.prediction_ids_[i]);
                    if(recognizer_.prediction_ids_[i] == desired_id)
                    {
                        ROS_INFO("Navigating!");
                        Rect face_roi = (recognizer_.detector_).detections_[i];         // retrieve face region
                        destination_reached_ = navigateToROI(face_roi.tl(), face_roi.br(), initial_navigation);
                        
                        if(initial_navigation)
                            initial_navigation = false;
                        
                        break;
                        //cout << destination_reached_ << endl;
                    }
                }
            }
            else if(strcmp(NAVIGATE, "body"))
            {
                ROS_INFO("Navigating!");
                Rect face_roi = (recognizer_.detector_).detections_[0];         // retrieve face region
                destination_reached_ = navigateToROI(face_roi.tl(), face_roi.br(), initial_navigation);
                
                if(initial_navigation)
                    initial_navigation = false;
                
                break;
                //cout << destination_reached_ << endl;
            }
        }
    }
    
    status_msg.data = "1";
    pub_personReached_.publish(status_msg);

}

void PowerBotNavigate::navigateToLocationCb(const std_msgs::Int32MultiArray::ConstPtr& coords)
{
    // initialuze navigation params to known values
    curr_target_.x = -1.0;
    curr_target_.z = -1.0;
    curr_target_.y = -1.0;
    waypoints_.clear();
    target_theta_ = 0.0;

    // get the desired coordinates from ROS msg
    int i = 0;
    for(std::vector<int>::const_iterator it = coords->data.begin(); it != coords->data.end(); it++)
    {
        if(i == 0)
            curr_target_.x = *it;
        else if(i == 1)
            curr_target_.z = *it;
        else if(i == 2)
            target_theta_ = *it;
        i++;
    } 

    // set y-val to arbitrary number
    curr_target_.y = 30.0;              // arbitrary

    // output for debugging
    stringstream ss;
    ss << "Received Command to navigate position: (" << curr_target_.x << " , " << curr_target_.z << ")\n";
    string rx_msg = ss.str();
    ROS_INFO(rx_msg.c_str());

    destination_reached_ = false;
    std_msgs::String status_msg;

    // first, navigate to the desired (x,y) coordinates
    if(pbClient_.getRunningWithLock())
    {

        pos_.x = curr_target_.x;
        pos_.z = curr_target_.z;

        pbClient_.requestUpdate();
        ROS_INFO("Current Location: {X,Y} = { %f, %f }", pbClient_.pbX, pbClient_.pbY);
        
        // transform destination into world frame
        /*pbClient_.transformPoints(pos_.x, pos_.z);
        ROS_INFO("Desired PowerBot Coordinates {X,Y} = { %f, %f }\n", pos_.x, pos_.z);*/

        ROS_INFO("Plotting New Course");
        // move powerbot to destination and busy wait  
        pbClient_.moveTo(pos_.x, pos_.z);
        usleep(5000000);

    
        ROS_INFO("Navigating to Destination");
        pbClient_.requestUpdate();
        // if powerbot's current position is close to the current position of the person, return
        //if((pbClient_.pbX <= curr_target_.x + 200 && pbClient_.pbX >= curr_target_.x - 200) && (pbClient_.pbY <= curr_target_.z + 200 && pbClient_.pbY >= curr_target_.z - 200))
        while((abs(pbClient_.pbX) >= abs(pos_.x) + 150 || abs(pbClient_.pbX) <= abs(pos_.x) - 150) || (abs(pbClient_.pbY) >= abs(pos_.z) + 150 || abs(pbClient_.pbY) <= abs(pos_.z) - 150))
        {
            // addded
            if(pbClient_.pbVel == 0.0)
            {
                ROS_INFO("Current Velocity: {Vel,LatVel} = { %f, %f }", pbClient_.pbVel, pbClient_.pbLatVel);
                pbClient_.moveTo(pos_.x, pos_.z);
                usleep(5000000);
            }

            pbClient_.requestUpdate();
            usleep(500000);
            ROS_INFO("Current Location: {X,Y} = { %f, %f }", pbClient_.pbX, pbClient_.pbY);
        }

        ROS_INFO("Within Proximity of Destination");    
        usleep(5000000);
        // rotate to face the proper direction
        pbClient_.requestUpdate();
        pbClient_.rotateTo(target_theta_);

        while((abs(pbClient_.pbTh) >= abs(target_theta_) + 7) || (abs(pbClient_.pbTh) <= abs(target_theta_) - 7))
        {
            pbClient_.requestUpdate();
            // addded
            if(pbClient_.pbRotVel == 0.0)
            {
                ROS_INFO("Current Velocity: {RotVel} = { %f}", pbClient_.pbRotVel);
                pbClient_.rotateTo(target_theta_);
                usleep(5000000);
            }
            usleep(500000);
            ROS_INFO("Current Heading: = %f", pbClient_.pbTh);
        }
        ROS_INFO("Desired Heading Reached");    
        
        usleep(5000);
        // flag a complete navigation to command node
        status_msg.data = "1";
        pub_locationReached_.publish(status_msg);
    }

}


void PowerBotNavigate::rotationCb(const std_msgs::Int32MultiArray::ConstPtr& coords)
{
    // initialuze navigation params to known values
    curr_target_.x = -1.0;
    curr_target_.z = -1.0;
    curr_target_.y = -1.0;
    waypoints_.clear();
    target_theta_ = 0.0;

    pbClient_.requestUpdate();
    ROS_INFO("Current Location: {X,Y} = { %f, %f }", pbClient_.pbX, pbClient_.pbY);
    // get the desired coordinates from ROS msg
    int i = 0;
    for(std::vector<int>::const_iterator it = coords->data.begin(); it != coords->data.end(); it++)
    {
        if(i == 0)
            curr_target_.x = pbClient_.pbX;
        else if(i == 1)
            curr_target_.z = pbClient_.pbY;
        else if(i == 2)
            target_theta_ = *it;
        i++;
    } 

    // set y-val to arbitrary number
    curr_target_.y = 30.0;              // arbitrary

    // output for debugging
    stringstream ss;
    ss << "Received Command to rotate: (" << target_theta_ << " degrees)\n";
    string rx_msg = ss.str();
    ROS_INFO(rx_msg.c_str());

    destination_reached_ = false;
    std_msgs::String status_msg;

    // first, navigate to the desired (x,y) coordinates
    if(pbClient_.getRunningWithLock())
    {

        pos_.x = curr_target_.x;
        pos_.z = curr_target_.z;

        ROS_INFO("Plotting New Course");

        // rotate to face the proper direction
        pbClient_.requestUpdate();
        pbClient_.rotateTo(target_theta_);

        while((abs(pbClient_.pbTh) >= abs(target_theta_) + 7) || (abs(pbClient_.pbTh) <= abs(target_theta_) - 7))
        {
            pbClient_.requestUpdate();
            usleep(500000);
            ROS_INFO("Current Heading: = %f", pbClient_.pbTh);
        }
        ROS_INFO("Desired Heading Reached");    
        
        usleep(5000);
        // flag a complete navigation to command node
        status_msg.data = "1";
        pub_rotationReached_.publish(status_msg);
    }

}



PowerBotNavigate::PowerBotNavigate()
{
    destination_reached_ = false;

    // setup Kinect and get a point cloud
    /*pcc_ = new PointCloudCapture("A00367801249047A");
    pcc_->startCapture();
    pcc_->getFrame(kin_data_);*/

    // connect to powerbot
    if(pbClient_.connect()) {
        ROS_INFO("Connected to PowerBot!\n");
    } else {
        ROS_INFO("Could not connect to PowerBot!\n");
        return;
    }

    /*********************************************************************************/
    //roi_sub_ = n_.subscribe("face_roi", 1, &PowerBotNavigate::navigateToROI, this);
    //ros::Rate loop_rate(30);
    /*********************************************************************************/

    // rotate away from table
    ROS_INFO("Getting PowerBot Pose...");
    pbClient_.requestUpdate();
    usleep(1000);

    /*
    stringstream ss;
    ss << "Current Heading: " << pbClient_.pbTh << "\n";
    string rx_msg = ss.str();
    ROS_INFO(rx_msg.c_str());


    ROS_INFO("Rotating Away from Table...");
    pbClient_.rotateTo(90.0 + pbClient_.pbTh);
    */

    ROS_INFO("Updating Recognizer...");
    recognizer_.update();         // update PCA model with training data
    ROS_INFO("Updating Complete!");
    ROS_INFO("Awaiting Coordinates...");


    // setup publishers/subscribers for navigation topics
    image_transport::ImageTransport it_(n_);
    pub_raw_ = it_.advertise("kinect/raw_image", 1);

    pub_personReached_ = n_.advertise<std_msgs::String>("Navigation/personReached", 1);
    pub_locationReached_ = n_.advertise<std_msgs::String>("Navigation/locationReached", 1);
    pub_rotationReached_ = n_.advertise<std_msgs::String>("Navigation/rotationReached", 1);

    // navigate to detection (receives id for face navigation or anything for detection navigation)
    sub_face_navigate_ = n_.subscribe("Navigation/faceID", 1, &PowerBotNavigate::navigateToDetectionCb, this);

    sub_location_navigate_ = n_.subscribe("Navigation/desiredLocation", 1, &PowerBotNavigate::navigateToLocationCb, this);
    sub_rotate_ = n_.subscribe("Navigation/desiredRotation", 1, &PowerBotNavigate::rotationCb, this);
    /*
    ros::Rate loop_rate(10);
    // publish kinect images
    while (ros::ok())
    {
        if( cv::waitKey(30) == 27 )
            return;
        
        // publish to rostopic 
        cv_bridge::CvImage cv_image;
        
        // Grab image for face detection from test image
        //cv_image.image = cv::imread("/home/kyle/catkin_ws/src/baxter_sp/src/test1.jpg",CV_LOAD_IMAGE_COLOR);
        
        // Grab image for face detection from kinect data
        
        pcc_->getFrame(kin_data_);
        imgWidth_ = kin_data_.srcImg.size().width;
        imgHeight_ = kin_data_.srcImg.size().height;
        
        // publish kinect image
        cv_image.image = kin_data_.srcImg;
        cv_image.encoding = "bgr8";
        sensor_msgs::Image ros_image;
        cv_image.toImageMsg(ros_image);
        pub_raw_.publish(ros_image);
        ROS_INFO("Published: kinect/image");
        cv::imshow("Published", cv_image.image);
        cv::waitKey(5);

        ros::spinOnce();
        loop_rate.sleep();
    }
    */

    ros::spin();
    //pcc_->stopCapture();
    

    return;
    // PUBLISH IMG
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "powerbot_navigate");
    PowerBotNavigate nav;
    ros::spin();
}