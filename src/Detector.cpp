
#include "Detector.h"

 using namespace std;
 using namespace cv;

Detector::Detector()
{
  window_name_ = "detection";
}

Detector::Detector(string xml_name){
      window_name_ = "detection";
      cascade_name_ = "/home/kyle/catkin_ws/src/baxter_sp/classifiers/" + xml_name;    // name of .xml classifier
};

Detector::~Detector(){};

void Detector::detect( Mat frame, bool show )
{

  viz_ = frame;
  if( !cascade_.load( cascade_name_ ) ){ printf("--(!)Error loading classifier\n"); return; };

  vector<Rect> detections;
  Mat frame_gray;

  cvtColor( frame, frame_gray, CV_BGR2GRAY );
  equalizeHist( frame_gray, frame_gray );

  // detect faces
  cascade_.detectMultiScale( frame_gray, detections, 1.2, 5, 0|CV_HAAR_SCALE_IMAGE, Size(20, 20) );
  detections_ = detections;     // update class variable
  detection_imgs_.clear();

  for( size_t i = 0; i < detections.size(); i++ )
  {
    Mat faceROI = frame_gray( detections[i] );
    cv::equalizeHist(faceROI, faceROI);
    detection_imgs_.push_back(faceROI);
    if(show)
    {
      rectangle(frame, detections[i], Scalar(255, 0, 0), 3);
    }
  }
  
  if(show)
    imshow( window_name_, frame );
 }


