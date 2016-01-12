#include "gestureTracker.h"

int clamp(int n, int min, int max);

gestureTracker::gestureTracker(ros::NodeHandle n): it(n)
{
	this->n = n;
	this->has_depth_data = false;
	this->l_hand_sub = this->n.subscribe("/hand_pos_2D/left", 1, &gestureTracker::l_hand_cb, this);
	this->r_hand_sub = this->n.subscribe("/hand_pos_2D/right", 1, &gestureTracker::r_hand_cb, this);

	this->r_hand_state_pub = this->n.advertise<std_msgs::Float64>("/gesture/hand_state/right", 1);
	this->l_hand_state_pub = this->n.advertise<std_msgs::Float64>("/gesture/hand_state/left", 1);

	this->l_roi_size_filter.resize(5);
	this->r_roi_size_filter.resize(5);
	this->r_hand_palm_x_filter.resize(7);
	this->r_hand_palm_y_filter.resize(7);
	this->l_hand_palm_x_filter.resize(7);
	this->l_hand_palm_y_filter.resize(7);
	
	this->l_hand_state_filter.resize(20);
	this->r_hand_state_filter.resize(20);

	this->in_use = false;
	this->depth_img_sub = this->it.subscribe("/camera/depth/image_raw", 1, &gestureTracker::depth_img_cb, this);
}

gestureTracker::~gestureTracker()
{

}

// subscribers retreive hand position on depth map for left and right hands of currently tracked user
void gestureTracker::l_hand_cb(const std_msgs::Int32MultiArray::ConstPtr& array)
{
	l_hand_pos_prev = l_hand_pos;

	std::vector<int>::const_iterator it = array->data.begin();
	this->l_hand_pos.x = *it;
	it++;
	this->l_hand_pos.y = *it;

	//cerr << " L: pos " << l_hand_pos.x  << " " << l_hand_pos.y << endl;

	if(abs(l_hand_pos_prev.x - l_hand_pos.x) > 10.0 || abs(l_hand_pos_prev.y - l_hand_pos.y) > 10.0)
		return;

	int w = 20;
	int h = 20;
	if(this->has_depth_data)
	{	
		//while(this->in_use)
		//	waitKey(1);
		this->in_use = true;
		//Mat depth;
		//depth = this->depth_img.clone();
		//this->depth_img.copyTo(depth);

		Size max = this->depth_img.size();
		int max_x = max.width;
		int max_y = max.height;

		int tl_x = clamp(l_hand_pos.x-w/2, 1, (max_x-w)-1);
		int tl_y = clamp(l_hand_pos.y-h/2, 1, (max_y-h)-1);

		/*
		cerr << " L: " << max_x << " " << max_y << endl;
		cerr << " L: " << l_hand_pos.x << " " << l_hand_pos.y << endl;
		cerr << " L: clamp in " << l_hand_pos.x-w/2 << " " << l_hand_pos.y-h/2 << endl;
		cerr << " L: clamp in " << l_hand_pos.x-(w/2) << " " << l_hand_pos.y-(h/2) << endl;
		cerr << " L: " << tl_x << " " << tl_y << " " << w << " " << h <<endl;
		cerr << " L: first roi" << endl;
		*/
		Rect roi = Rect(tl_x, tl_y, w, h);
		Scalar avg = cv::mean(this->depth_img(roi));

		//cerr << " L: first roi success" << endl;

		double scale_w = (avg[0]/(double)12750);
		double scale_h = (avg[0]/(double)12750);

		if(scale_w <= 0 || scale_h <= 0)
			return;

		int scaled_w = 1/scale_w * 110 + 60;
		int scaled_h = 1/scale_h * 110 + 60;

		//cerr << " L: " << avg[0] << " " << scaled_w << " " << scale_h << endl;

		scaled_h = clamp(scaled_h, scaled_h, 200);
		scaled_w = clamp(scaled_w, scaled_w, 200);

		l_roi_size_filter.add(scaled_w);
		scaled_w = l_roi_size_filter.average();
		scaled_h = scaled_w;

		int tl_x_new = clamp(l_hand_pos.x-scaled_w/2, 0, (max_x-scaled_w));
		int tl_y_new = clamp(l_hand_pos.y-scaled_h/2, 0, (max_y-scaled_h));

		/*
		cerr << " L: " << tl_x_new << " " << tl_y_new << " " << scaled_w << " " << scaled_h << endl;
		cerr << " L: scaling roi" << endl;
		*/
		Rect roi_scaled = Rect(tl_x_new, tl_y_new, scaled_w, scaled_h);
		this->depth_img(roi_scaled).copyTo(l_hand_roi);
		
		this->in_use = false;
		//imshow("left", l_hand_roi);
		//waitKey(60);
		detectGesture(l_hand_roi, "left");
	}

}

void gestureTracker::r_hand_cb(const std_msgs::Int32MultiArray::ConstPtr& array)
{
	r_hand_pos_prev = r_hand_pos;

	std::vector<int>::const_iterator it = array->data.begin();
	this->r_hand_pos.x = *it;
	it++;
	this->r_hand_pos.y = *it;

	//cerr << "R: pos " << r_hand_pos.x  << " " << r_hand_pos.y << endl;

	if(abs(r_hand_pos_prev.x - r_hand_pos.x) > 10.0 || abs(r_hand_pos_prev.y - r_hand_pos.y) > 10.0)
		return;

	int w = 250;
	int h = 250;
	if(this->has_depth_data)
	{
		//while(this->in_use)
		//	waitKey(1);
		this->in_use = true;
		Mat depth;
		//depth = this->depth_img.clone();
		//this->depth_img.copyTo(depth);

		Size max = this->depth_img.size();
		int max_x = max.width;
		int max_y = max.height;

		int tl_x = clamp(r_hand_pos.x-w/2, 1, (max_x-w)-1);
		int tl_y = clamp(r_hand_pos.y-h/2, 1, (max_y-h)-1);
		/*
		cerr << "R: " << max_x << " " << max_y << endl;
		cerr << "R: clamp in " << r_hand_pos.x-w/2 << " " << r_hand_pos.y-h/2 << endl;
		cerr << "R: " << tl_x << " " << tl_y << " " << w << " " << h << endl;
		cerr << "R: first roi" << endl;
		*/
		Rect roi = Rect(tl_x, tl_y, w, h);
		Scalar avg = cv::mean(this->depth_img(roi));

		//cerr << "R: first roi success" << endl;

		double scale_w = (avg[0]/(double)12750);
		double scale_h = (avg[0]/(double)12750);
		int scaled_w = 1/scale_w * 110 + 60;
		int scaled_h = 1/scale_h * 110 + 60;

		if(scale_w <= 0 || scale_h <= 0)
			return;

		//cerr << "R: " << avg[0] << " " << scaled_w << " " << scale_h << endl;

		scaled_h = clamp(scaled_h, scaled_h, 200);
		scaled_w = clamp(scaled_w, scaled_w, 200);

		r_roi_size_filter.add(scaled_w);
		scaled_w = r_roi_size_filter.average();
		scaled_h = scaled_w;

		int tl_x_new = clamp(r_hand_pos.x-scaled_w/2, 0, (max_x-scaled_w));
		int tl_y_new = clamp(r_hand_pos.y-scaled_h/2, 0, (max_y-scaled_h));

		/*
		cerr << "R: " << tl_x_new << " " << tl_y_new << " " << scaled_w << " " << scaled_h << endl;
		cerr << "R: scaling roi" << endl;
		*/

		Rect roi_scaled = Rect(tl_x_new, tl_y_new, scaled_w, scaled_h);
		this->depth_img(roi_scaled).copyTo(r_hand_roi);

		this->in_use = false;
		//imshow("right", r_hand_roi);
		//waitKey(60);

		detectGesture(r_hand_roi, "right");
	}
}

void gestureTracker::depth_img_cb(const sensor_msgs::ImageConstPtr& msg_ptr)
{
	try
	{
		this->depth_img = cv_bridge::toCvShare(msg_ptr, "16UC1")->image;
		cv::normalize(this->depth_img, this->depth_img, 0, 65535, NORM_MINMAX, CV_16UC1);
		//cv::imshow("depth", this->depth_img);
		//cv::waitKey(30);
		this->has_depth_data = true;
	}
	catch (cv_bridge::Exception& e)
	{
		this->has_depth_data = false;
		ROS_ERROR("Could not convert from '%s' to '16UC1'.", msg_ptr->encoding.c_str());
	}	
}

void gestureTracker::detectGesture(Mat hand_roi, string rl)
{
	Mat thresh;
	Mat img = Mat::zeros( hand_roi.size(), CV_8UC3 );
	img = hand_roi.clone();
	
	Size max = hand_roi.size();
	int max_x = max.width;
	int max_y = max.height;

	double palm_dist = 0.0;

	Rect thresh_roi = Rect(3*max_x/7, 3*max_y/7, max_x/7, max_y/7);
	Scalar avg = cv::mean(img(thresh_roi));
	//cerr << avg[0] << endl;

	int max_dist = avg[0] + 1400;
	int min_dist = 0;
	if(avg[0] >= 1400)
		min_dist = avg[0] - 1400;
	
	for (int i = 0; i < max_y; i++)
    {
        for (int j = 0; j < max_x; j++)
        {
            int val = (int)img.at<short>(i,j);
            if(val <= min_dist)
            	img.at<short>(i,j) = 0;
            else if(val >= max_dist)
            	img.at<short>(i,j) = 0;
        }	
    }

    img.convertTo(img, CV_8UC3);

    vector<vector<Point> > contours;
  	vector<Vec4i> hierarchy;
    findContours( img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    
     /// Draw contours
    double max_area = 0.0;
    int idx = -1;
	Mat drawing = Mat::zeros( img.size(), CV_8UC3 );
	//cerr << contours.size() << endl;
	for( int i = 0; i < contours.size(); i++ )
	{
		double area = contourArea(contours[i]);
		if(area > max_area )
		{
			max_area = area;
			idx = i;
		}
	}

	Scalar white = Scalar( 255, 255, 255 );
	Scalar yellow = Scalar( 0, 255, 255 );
	Scalar purple = Scalar( 255, 0, 255 );
	Scalar blue = Scalar( 255, 0, 0 );
	Scalar red = Scalar( 0, 0, 255 );
	Scalar green = Scalar( 0, 255, 0 );
	
	if(idx != -1)
	{
		//drawContours( drawing, contours, idx, white, 1, 8, vector<Vec4i>(), 0, Point() );
		//imshow(rl, drawing);
		//cv::waitKey(20);
	}
		
	
	if(idx != -1)
	{
		if (contours[idx].size() <= 5)
			return;

		//cerr << idx << endl;	
		// convex hull for drawing (Points)
		vector<vector<Point> > hullp(contours[idx].size());
		// convex Hull for defects (ints)
		vector<vector<int> > hulli(contours[idx].size());
		vector<vector<Point> > hull_points(contours[idx].size());
        vector<vector<Point> > defect_points(contours[idx].size());

		convexHull( Mat(contours[idx]), hulli[0], false );
		convexHull( Mat(contours[idx]), hullp[0], false );

		/*
		for(int i = 0; i < hullp[0].size(); i++)
			circle( drawing, hullp[0][i], 5, blue, -1 );
	 	*/

	 	vector<vector<Vec4i> > defects(contours[idx].size());
	 	if (hulli[0].size() > 2 && contours[idx].size() > 3)
	 	{
	 		convexityDefects(contours[idx], hulli[0], defects[0]);

	 		Point palm;
			double x_sum = 0.0;
			double y_sum = 0.0;
	 		for(int i = 0; i < defects[0].size(); i++)
		 	{
		 		for(int j = 0; j < 3; j++)
		 		{
		 			x_sum += contours[idx][defects[0][i][j]].x;
		 			y_sum += contours[idx][defects[0][i][j]].y;
		 		}
		 	}
		 	
	 		palm.x = (x_sum / defects[0].size()) / 3.0;
	 		palm.y = (y_sum / defects[0].size()) / 3.0;

	 		if(rl == "right")
	 		{
	 			r_hand_palm_x_filter.add(palm.x);
	 			r_hand_palm_y_filter.add(palm.y);
	 			palm.x = r_hand_palm_x_filter.average();
	 			palm.y = r_hand_palm_y_filter.average();	
	 		}
	 		else
	 		{
		 		l_hand_palm_x_filter.add(palm.x);
	 			l_hand_palm_y_filter.add(palm.y); 
	 			palm.x = l_hand_palm_x_filter.average();
	 			palm.y = l_hand_palm_y_filter.average();			
	 		}

	 		// max = 20 ish, min  = 7 ish
	 		palm_dist = (hand_roi.at<short>(palm.y, palm.x)/65536.0)*100.0;
	 		cerr << "Palm: " << palm_dist << endl;

	 		circle(drawing, palm, 5, green, -1);
	 		double avg_dist = 0.0;
		 	for(int i = 0; i < defects[0].size(); i++)
		 	{
		 		int idx_0 = defects[0][i][0];
		 		int idx_1 = defects[0][i][1];
		 		int idx_2 = defects[0][i][2];
		 		defect_points[i].push_back(contours[idx][idx_2]);
		 		
		 		/*
		 		circle(drawing, contours[idx][idx_0], 5, yellow, -1);
		 		circle(drawing, contours[idx][idx_1], 5, blue, -1);
		 		circle(drawing, contours[idx][idx_2], 5, red, -1);
		 		line(drawing, contours[idx][idx_2], contours[idx][idx_0], purple, 1);
		 		line(drawing, contours[idx][idx_2], contours[idx][idx_1], purple, 1);
				*/

		 		int tip_x = contours[idx][idx_0].x;
		 		int tip_y = contours[idx][idx_0].y;

		 		int defect_x = contours[idx][idx_2].x;
		 		int defect_y = contours[idx][idx_2].y;

		 		double dist = 0.0;
		 		dist = sqrt(pow((tip_x - defect_x), 2) + pow((tip_y - defect_y), 2));
		 		avg_dist += dist;
		 	}

		 	if(defects[0].size() > 0)
		 	{
		 		avg_dist = avg_dist / defects[0].size();
		 		// normalize for fingers
		 		avg_dist = avg_dist / 20.0;
		 	 	cerr << rl << " " << defects[0].size()/3 - 2 << endl;

		 	 	std_msgs::Float64 state_msg;
			 	if(rl == "right")
			 	{
			 		r_hand_state_filter.add(avg_dist - 0.3);
			 		state_msg.data = (r_hand_state_filter.average());
			 		r_hand_state_pub.publish(state_msg);
			 		//cerr << "distance R: " << r_hand_state_filter.average() << endl;

			 	}
			 	else
			 	{

			 		l_hand_state_filter.add(avg_dist - 0.3);
			 		state_msg.data = (l_hand_state_filter.average());
			 		l_hand_state_pub.publish(state_msg);
			 		//cerr << "distance L: " << l_hand_state_filter.average() << endl;
			 	}
		 	}

	 	}
	 	/*
	 	try{
	 	if(rl == "right")
			imshow(rl + "hand", drawing);
		else
			imshow(rl + "hand", drawing);
		}
		catch(...){return;}
		*/

		cv::waitKey(20);
	//threshold(img, thresh, 30, 255, CV_THRESH_BINARY_INV);
	
	}
	
}

int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "gestureTracker");
	ros::NodeHandle n;
	ros::Rate loop_rate(160);

	gestureTracker gt(n);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

int clamp(int n, int min, int max) {
  if(n > max)
  	return max;
  else if(n < min)
  	return min;
  else
  	return n;
}