#include "OCR.h"

using namespace cv;
using namespace std;

void transform_if_first_of_word( char& c )
{
    // if the previous character was a space, transform it toupper
    if( (*(&c - sizeof(char))) == ' ')
        c = toupper( c );
}

void OCR::readText()
{
    char *outText;

    tesseract::TessBaseAPI *api = new tesseract::TessBaseAPI();
    // Initialize tesseract-ocr with English, without specifying tessdata path
    if (api->Init(NULL, "eng")) {
        fprintf(stderr, "Could not initialize tesseract.\n");
        exit(1);
    }

    for(int i = 0; i < this->textROIs.size(); i++)
    {
    	api->SetImage((uchar*)this->textROIs[i].data, this->textROIs[i].size().width, this->textROIs[i].size().height, this->textROIs[i].channels(), this->textROIs[i].step1());
    	// Get OCR result
    	outText = api->GetUTF8Text();
    	string str = std::string(outText);
    	this->textStrings.push_back(str);
    }
    // Destroy used object and release memory
    api->End();

}

void OCR::detectLetters()
{
	this->img.copyTo(this->results);
	this->textROIs.clear();
	this->textRects.clear();
	this->textStrings.clear();

    cv::Mat img_gray, img_sobel, img_threshold, element;
    cvtColor(this->img, img_gray, CV_BGR2GRAY);
    cv::Sobel(img_gray, img_sobel, CV_8U, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
    cv::threshold(img_sobel, img_threshold, 0, 255, CV_THRESH_OTSU+CV_THRESH_BINARY);
    //element = getStructuringElement(cv::MORPH_RECT, cv::Size(29, 7) );
    //element = getStructuringElement(cv::MORPH_RECT, cv::Size(15, 3) );
    element = getStructuringElement(cv::MORPH_RECT, cv::Size(this->k1, this->k2) );
    cv::morphologyEx(img_threshold, img_threshold, CV_MOP_CLOSE, element); //Does the trick
    std::vector< std::vector< cv::Point> > contours;
    cv::findContours(img_threshold, contours, 0, 1); 
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );

    cv::Mat roi;
    for( int i = 0; i < contours.size(); i++ )
    {
        if (contours[i].size()>100)
        { 
            cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
            cv::Rect appRect( boundingRect( cv::Mat(contours_poly[i]) ));
            if (appRect.width>appRect.height*3) 
            {
            	this->textRects.push_back(appRect);
            	roi = this->img(appRect);
                this->textROIs.push_back(roi);
            }
        }
    }

    // save files for OCR
    for(int j = 0; j < this->textROIs.size(); j++)
    {
        cv::rectangle(this->results,this->textRects[j],cv::Scalar(0,0,255),3,8,0);
    }

    readText();

    if(VERBOSE)
		for(int i = 0; i < this->textStrings.size(); i++)
		{
			cout << this->textStrings[i];
		}

	findMatch();
}

// wikibooks: http://en.wikibooks.org/wiki/Algorithm_Implementation/Strings/Longest_common_substring
int OCR::LongestCommonSubstring(const string& str1, const string& str2)
{
     if(str1.empty() || str2.empty())
     {
          return 0;
     }
 
     int *curr = new int [str2.size()];
     int *prev = new int [str2.size()];
     int *swap = NULL;
     int maxSubstr = 0;
 
     for(int i = 0; i<str1.size(); ++i)
     {
          for(int j = 0; j<str2.size(); ++j)
          {
               if(str1[i] != str2[j])
               {
                    curr[j] = 0;
               }
               else
               {
                    if(i == 0 || j == 0)
                    {
                         curr[j] = 1;
                    }
                    else
                    {
                         curr[j] = 1 + prev[j-1];
                    }
                    //The next if can be replaced with:
                    //maxSubstr = max(maxSubstr, curr[j]);
                    //(You need algorithm.h library for using max())
                    if(maxSubstr < curr[j])
                    {
                         maxSubstr = curr[j];
                    }
               }
          }
          swap=curr;
          curr=prev;
          prev=swap;
     }
     delete [] curr;
     delete [] prev;
     return maxSubstr;
}

void OCR::findMatch()
{
	int longest_match_len = 0;
	int current_match_len = 0;
	int current_match_idx = 0;
	int desired_string_idx = 0;
	for(int i = 0; i < this->textStrings.size(); i++)
	{
		for(int s = 0; s < this->desired_titles.size(); s++)
		{
			current_match_len = LongestCommonSubstring(this->desired_titles[s], this->textStrings[i] );
			if(current_match_len > longest_match_len)
			{
				longest_match_len = current_match_len;
				current_match_idx = i;
				desired_string_idx = s;
			}
		}
	}

	//if(longest_match_len >= this->desired_titles[desired_string_idx].length() - this->desired_titles[desired_string_idx].length()/4)
	if(longest_match_len >= this->desired_titles[desired_string_idx].length()/4)
	{
		this->detected_match = longest_match_len;
		cv::rectangle(this->results,this->textRects[current_match_idx],cv::Scalar(0,255,0),3,8,0);

		cv::Point tl = this->textRects[current_match_idx].tl();
		int c_x = (tl.x + this->textRects[current_match_idx].width/2);
		int c_y = (tl.y + this->textRects[current_match_idx].height/2);


		circle( this->results, Point( c_x, c_y ), 4.0, Scalar( 0, 255, 0), 2, 4 );
		this->center.x = c_x;
		this->center.y = c_y;

		if(VERBOSE)
		{
			cout << "Match Len: " << longest_match_len << " text: " << this->textStrings[current_match_idx] << endl;
			cout << current_match_idx << endl;
			imshow("detection", this->results);
			cv::waitKey(30);
		}
	}
	else
	{
		this->detected_match = -1;
	}
}





OCR::~OCR()
{
	
}

OCR::OCR(ros::NodeHandle n): it(n)
{
	this->center.x = -1;
	this->center.y = -1;
	
	ROS_INFO("Initializing OCR");
	
	this->pub_bookLocation = this->n.advertise<std_msgs::Int32MultiArray>("OCR/bookLocation", 1);
	this->sub_bookTitle = this->n.subscribe("OCR/bookTitle", 1, &OCR::bookTitleCb, this);

	this->sub_rh_image = this->it.subscribe("cameras/right_hand_camera/image", 1, &OCR::rhImageCb, this);
	this->sub_lh_image = this->it.subscribe("cameras/left_hand_camera/image", 1, &OCR::lhImageCb, this);
	
	ros::spin();

    //cv::imwrite( "/home/kyle/catkin_ws/src/baxter_sp/src/imgOut1.jpg", img1);  
}

void OCR::rhImageCb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
  	this->rh_img= cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::imshow("right hand", this->rh_img);
    //cout << "image size: " << this->rh_img.rows << " " << this->rh_img.cols << endl;

    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void OCR::lhImageCb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
  	this->lh_img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::imshow("left hand", this->lh_img);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


void OCR::bookTitleCb(const std_msgs::String::ConstPtr& msg)
{
	bool bookFound = false;
	int rl = 1;			// right=0, left = 1

	while(!bookFound && this->rh_img.rows > 0 && this->lh_img.rows > 0)
	{
		this->center.x = -1;
		this->center.y = -1;
		this->textStrings.clear();
		this->textRects.clear();
		this->textROIs.clear();
		this->desired_titles.clear();

	    //Read
		this->k1 = 23;
		this->k2 = 7;
		
		string s1, s2, s3;

		// must be lowercase string from phone
		//s1 = "mean streak";
		s1 = msg->data.c_str();
		s2 = s1;
		s3 = s1;

		for(unsigned int k = 0; k < s1.length(); k++)
		{
			s1[k] = toupper(s1[k]);
		}

		s2[ 0 ] = toupper( s2[ 0 ]);
	   	std::for_each( s2.begin()+1, s2.end(), transform_if_first_of_word );

	   	this->desired_titles.push_back(s1);
		this->desired_titles.push_back(s2);
		this->desired_titles.push_back(s3);

		cout << "looking for: " + s1 + " or " + s2 + " or " + s3 << endl;
		// detect the title

		Mat temp;
		//temp.create(720, 1280, CV_8UC3);
		if(rl == 0)
			this->img = this->rh_img;
		else
			this->img = this->lh_img;

		transpose(this->img, this->img);
		flip(this->img, this->img, 0);
	

		detectLetters();
		imshow("results", this->results);
		cv::waitKey(30); 

		std_msgs::Int32MultiArray book_pos;
		//Clear array
		book_pos.data.clear();
		//for loop, pushing data in the size of the array
		for (int i = 0; i < 2; i++)
		{
			book_pos.data.push_back(this->center.x);
			book_pos.data.push_back(this->center.y);
		}

		if(this->center.x != -1 && this->center.y != -1)
		{
			ROS_INFO("Found the book!");
			this->pub_bookLocation.publish(book_pos); 
			bookFound = true;
		}
		else
		{
			ROS_INFO("No Book Found");
		}
		ros::spinOnce();
		waitKey(30);

		// comment out for testing
		bookFound = true;
	}
    
}

int main(int argc,char** argv)
{
	ros::init(argc, argv, "OCR");
	ros::NodeHandle n;
    OCR ocr = OCR(n);
    ros::spin();
}