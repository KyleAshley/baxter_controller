#include <iostream>
#include <fstream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/cloud_viewer.h>
#include "PointCloudCapture.h"
#include <string>

using namespace std;

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr srcCloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr srcCloud3 (new pcl::PointCloud<pcl::PointXYZRGBA>);

int count2 = 1;
cv::Point mousePointerPos2;
//std::ofstream pointsFs("points.txt");

std::fstream pointsFs;

///This function gets the position from text input
vector<double> getPos() {
    bool isset = false;
    double x,y,z;
    std::vector<double> pos;
    string input;
    std::vector<string> tokens;

    while(!isset) {
        cout << "Enter Coordinates in X;Y;Z Format: ";
        getline(cin, input);
        boost::algorithm::split(tokens, input, boost::is_any_of(";"));

        if(tokens.size() != 3) {
            cout << "Incorrect format!" << "\n";
        } else {
            isset = true;
            x = atof(tokens[0].c_str());
            y = atof(tokens[1].c_str());
            z = atof(tokens[2].c_str());
        }
    }

    pos.resize(3);
    pos[0] = x;
    pos[1] = y;
    pos[2] = z;
    return pos;
}

bool readMatfromFile(std::ifstream &matFile, Eigen::Matrix4f &mat){
    char temp_buf[200];
    double temp[4];
    for(int i =0; i < 4; i++) {
        matFile.getline(temp_buf,200);
        if (matFile.eof()) break;
        if (sscanf(temp_buf,"%lf %lf %lf %lf", &(temp[0]), &(temp[1]), &(temp[2]), &(temp[3])) == 4) {
            for(int j=0;j<4;j++)
                mat(i,j) = temp[j];
        }
        else {
            return false;
        }
    }
    return true;
}

//test for readMatfromFile() function
void test_readMatfromFile(){
    Eigen::Matrix4f trans;
    std::ifstream matFile;
    matFile.open("mat.txt", std::fstream::in /*| std::fstream::out | std::fstream::app*/);
    if(readMatfromFile(matFile,trans)) {
        cout << "Transformation read is :" <<endl;
        cout << trans << endl;
    }
}

void calculateTransform(std::fstream &pointsFs) {
    Eigen::Matrix4f trans;
    pcl::PointCloud<pcl::PointXYZ> points_in_kinect;
    pcl::PointCloud<pcl::PointXYZ> points_in_arm;

    //read data from file into data structures
    char temp_buf[200];
    pcl::PointXYZ KinPos;
    pcl::PointXYZ ArmPos;
    while(true){
        pointsFs.getline(temp_buf,200);
        if(pointsFs.eof()) break;
        if (sscanf(temp_buf,"%f,%f,%f,%f,%f,%f", &(KinPos.x), 
                   &(KinPos.y), &(KinPos.z), &(ArmPos.x), &(ArmPos.y), 
                   &(ArmPos.z)) == 6) {
            points_in_kinect.push_back(KinPos);
            points_in_arm.push_back(ArmPos);
        }
    }

    //calibrate
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> transformation_estimator;
    if(points_in_kinect.size() > 6 ) {
        transformation_estimator.estimateRigidTransformation(points_in_kinect,points_in_arm,trans);
        cout << "Calculated transformation: " <<endl;
        cout << trans << endl;
        //write to file
        std::ofstream fs("mat.txt");
        if(!fs){
            std::cerr<<"Cannot open the output file."<<std::endl;
        }
        else{
            fs<< trans << endl ;
            fs.close();
        }
        cout << "transformation matrix written to mat.txt " << endl;
    }
    else{
        cout << "Not enough points to run SVD estimation" << endl;
    }
}

void on_mouse2(int eve, int x, int y, int flags,void* p) {
    if (eve == 1) {
        //mouseInterupt = true;
        mousePointerPos2 = cv::Point(x,y);
        //get pos from point cloud
        int h = y;
        int w = x;
        int validPoint = 0;
        pcl::PointXYZRGBA pos = srcCloud2->points[h* srcCloud2->width + w];
        cout << count2 <<": " << x << " " << y << " " << pos.x << " " << pos.y << " " << pos.z << endl;
        //get position from wheelchair code
        vector<double> armPos = getPos();
        cout << "Arm Position " << armPos[0] << " " << armPos[1] << " " << armPos[2] << endl;

        cout << "would you like to use the current point for calculation? " <<endl;
        cout << "press 1 for 'YES' and 0 for 'NO' : " ;
        cin >> validPoint;
        if(validPoint == 1) {
            //cout << "received " << "1" << endl;
            // write to a data structure
            pcl::PointXYZ kPos(pos.x,pos.y,pos.z);
            pcl::PointXYZ aPos(armPos[0],armPos[1],armPos[2]);
            //points_in_kinect.push_back(kPos);
            //points_in_arm.push_back(aPos);
            pointsFs << pos.x << "," << pos.y << "," << pos.z << "," << armPos[0] << "," << armPos[1] << "," << armPos[2] << endl;
            cout  << "position added to dataset and written to points.txt file" << endl;
        }
        count2++;
    }
    return;
}

int main(int argc, char *argv[]) {
    //test_readMatfromFile();

    /*
    * Capture point cloud
    */
    pointsFs.open("points.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    Eigen::Matrix4f transformation;
    //CalibrateKinectWMRA(trans);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr srcCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    PointCloudCapture* cap = new PointCloudCapture();
    cap->startCapture();
    cap->getFrame(srcCloud2);

    cv::Mat srcImg;
    srcImg.create(cv::Size(srcCloud2->width,srcCloud2->height), CV_8UC3);
    cout << "format: image x, image y, point x, point y, pointz" << endl;

    //PointCloudCapture::makeImageFromPointCloud(srcImg, *srcCloud2);
    cv::namedWindow( "SOURCEIMAGE", CV_WINDOW_AUTOSIZE );
    //cv::imshow("SOURCEIMAGE", srcImg);
    cv::setMouseCallback("SOURCEIMAGE", on_mouse2);

    cout << "Entering Point Capture loop.... \nPlease focus on video window and press q to exit this mode" <<endl;
    char ch;
    do {
        cap->getFrame(srcCloud2);
        PointCloudCapture::makeImageFromPointCloud(srcImg, *srcCloud2);
        cv::imshow("SOURCEIMAGE", srcImg);
        ch = cv::waitKey(10);
    } while(ch != 'q');

    //ask user if they would like to calculate the transformation matrix
    int flag;
    do {
        cout << "would you like to run the calibration now? 1)Yes 2)No : " ;
        cin >> flag ;
    } while( flag != 1 && flag != 2);

    if ( flag == 1) { // create a file and write to it
        calculateTransform(pointsFs);
    }

    cout << "\nPress any key to close program" << endl;
    cin.get();
    return 0;
}
