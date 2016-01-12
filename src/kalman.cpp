
double pos_x, pos_y, pos_z, orient_x, orient_y, orient_z;

void poseCallback(const geometry_msgs::Quaternion::Ptr& msg)
{
  ROS_INFO("Received: [%d %d %d, %d %d %d]", msg->data.position.x, msg->data.position.y, msg->data.position.z, msg->data.orientation.x, msg->data.orientation.y, msg->data.orientation.z);
  pos_x = msg->data.position.x;
  pos_y = msg->data.position.y;
  pos_z = msg->data.position.z;

  orient_x = msg->data.orientation.x;
  orient_y = msg->data.orientation.y;
  orient_z = msg->data.orientation.z;
}

void main()
{
	ros::init(argc, argv, "listener");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("camera_pose", 1, poseCallback);

	
	cv::KalmanFilter KF;         // instantiate Kalman Filter

	int nStates = 18;            // the number of states
	int nMeasurements = 6;       // the number of measured states
	int nInputs = 0;             // the number of action control

	double dt = 0.125;           // time between measurements (1/FPS)

	initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);    // init function

	ros::spin();
}

