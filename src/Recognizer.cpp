#include "Recognizer.h"

using namespace cv;
using namespace std;



static Mat norm_0_255(InputArray _src);

static Mat norm_0_255(InputArray _src) {
    Mat src = _src.getMat();
    // Create and return normalized image:
    Mat dst;
    switch(src.channels()) {
    case 1:
        cv::normalize(_src, dst, 0, 255, NORM_MINMAX, CV_8UC1);
        break;
    case 3:
        cv::normalize(_src, dst, 0, 255, NORM_MINMAX, CV_8UC3);
        break;
    default:
        src.copyTo(dst);
        break;
    }
    return dst;
}

// constructor
Recognizer::Recognizer(){
    profile_directory_ = "/home/baxter/ros/ws_carrt/src/baxter_controller/people";
    if(NAVIGATE == "face")
    {
        cout << "Navigation set to faces" << endl;
        detector_ = Detector("haarcascade_frontalface_default.xml");
    }
    else if(NAVIGATE == "body")
    {
        detector_ = Detector("haarcascade_upperbody.xml");
    }

    model_ = createEigenFaceRecognizer(80, 3400.0);
    num_profiles_ = 4;
    

    //model_ = createFisherFaceRecognizer(180, 10);
    
};

// destructor
Recognizer::~Recognizer(){};

// reads a single image from profile directory
Mat Recognizer::readProfileImg(int profile_num, int img_num)
{
    Mat img;
    stringstream ss;
    ss << profile_directory_ << "/person_" << profile_num << "/training_img" << img_num << ".jpg";
    string filename = ss.str();
    img = imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    cv::resize(img, img, Size(60, 60), 1.0, 1.0, INTER_CUBIC);
    cv::equalizeHist(img, img);

    return img;
}

// update PCA model
void Recognizer::update()
{
    // read training images
    Mat training_img;
    for(int i = 1; i < num_profiles_ + 1; i++)
    {
        stringstream ss;
        ss << profile_directory_ << "/person_" << i;
        string dir_path = ss.str();
        cout << "Reading: " << dir_path << endl;
        int count = static_cast<int>(std::distance(boost::filesystem3::directory_iterator(dir_path), boost::filesystem3::directory_iterator()));
        for (int j = 0; j < count; j++)
        {
            training_img = readProfileImg(i, j);
            images_.push_back(training_img);
            labels_.push_back(i);
        }
    }
    cout << "Training..." << endl;
    model_->train(images_, labels_);
    cout << "Done!" << endl;

}


void Recognizer::recognizeDetections(Mat kin_img, bool show)
{
    Mat img;
    kin_img.copyTo(img);
    if( !img.empty() )
    { 
        detector_.detect( img, true ); 
    }

    detector_.viz_.copyTo(viz_);                // copy detector visualization
    prediction_ids_.clear();

    if(detector_.detections_.size() > 0)
    {
        cout << detector_.detections_.size() << " detected!" << endl;
        
        Mat detected_region;
        int prediction_num;
        double confidence;

        for(int i = 0; i < detector_.detections_.size(); i++){
            cv::resize(detector_.detection_imgs_[i], detected_region, Size(60, 60), 1.0, 1.0, INTER_CUBIC);
            model_->predict(detected_region, prediction_num, confidence);
            cout << "Recognized: " << prediction_num << " Confidence: " << confidence << endl;
            stringstream ss;
            ss << prediction_num;
            string id_text = ss.str();

            // if a person was successfully recognized draw the recognition roi
            if(prediction_num != -1)
            {
                prediction_ids_.push_back(prediction_num);
                rectangle(viz_, detector_.detections_[i], Scalar(255, 0, 0), 3);
                putText(viz_, id_text,  (detector_.detections_[i]).tl(), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 3, 8 );
            }
            if(show)
                imshow("recognizer", viz_);
        }   
    }

}


/*
int main(int argc, char **argv)
{
    CvCapture* capture;
    Mat frame;

    Recognizer r;
    //r.update();
    capture = cvCaptureFromCAM( -1 );
    frame = cvQueryFrame( capture );
    while( cv::waitKey(30) != 27 )
    {
        if( !frame.empty() )
        {
            r.recognizeDetections(frame, true);
        }
        frame = cvQueryFrame( capture );
    }

    cvReleaseCapture(&capture);
    return 0;
}
*/

