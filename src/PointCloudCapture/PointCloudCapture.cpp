#include "PointCloudCapture/PointCloudCapture.h"
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <string>

PointCloudCapture::PointCloudCapture() : ptCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>), 
    filteredPtCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>), initialized(false)
{ 
    needCloud = false;
    cloudDataReady = false;
    multipleKinect = false;
}

PointCloudCapture::PointCloudCapture(std::string _kinectID) : ptCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>), 
    filteredPtCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>), initialized(false)
{ 
    needCloud = false;
    cloudDataReady = false;
    multipleKinect = false;
    if(_kinectID != "") {
        kinectID = _kinectID;
        multipleKinect = true;
    }
}

PointCloudCapture::~PointCloudCapture() {
    if(initialized) kinInterface->stop();	
}

void PointCloudCapture::cloudCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
    if(needCloud){
        *ptCloudPtr = *cloud;
        needCloud = false;
        cloudDataReady = true;
    }
}

void PointCloudCapture::startCapture() {
    //Support for Multiple Kinects
    if(multipleKinect) {
        kinInterface = new pcl::OpenNIGrabber(kinectID);  // create instance of grabber
    } else {
        kinInterface = new pcl::OpenNIGrabber();
    }

    //#fix should check if a valid interface is returned. implement later
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> 
        f = boost::bind(&PointCloudCapture::cloudCallback, this, _1);

    kinInterface->registerCallback(f);
    kinInterface->start();
    initialized = true;
    usleep(1000000); //Wait for the initial data I guess
}

void PointCloudCapture::stopCapture(){
    kinInterface->stop();
    initialized = false;
}

void PointCloudCapture::getFrame(InputData& data) {
    cloudDataReady = false;
    needCloud = true;
    while(!cloudDataReady) usleep(1000); // wait for data to be ready
    //then copy point cloud
    pcl::copyPointCloud(*ptCloudPtr, *data.srcCloud );
    needCloud = false;
    cloudDataReady = false;
    //create a cv::img from the point cloud
    makeImageFromPointCloud(data.srcImg, *data.srcCloud);
}

void PointCloudCapture::getFrame(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptCloud) {
    cloudDataReady = false;
    needCloud = true;
    while(!cloudDataReady) usleep(1000); // wait for data to be ready
    //then copy point cloud
    pcl::copyPointCloud(*ptCloudPtr, *ptCloud);
    needCloud = false;
    cloudDataReady = false;

    // Only for debugging purposes
    if (false) {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Debugging Viewer"));
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(ptCloud);
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZRGBA> (ptCloud, rgb, "Filtered Cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Filtered Cloud");
        viewer->initCameraParameters();
        viewer->resetCameraViewpoint("Filtered Cloud");

        while (!viewer->wasStopped()) {
            viewer->spinOnce();
            boost::this_thread::sleep (boost::posix_time::microseconds(10));
        }	

        //pcl::io::savePLYFile("CapturedTestCloud.ply", *filteredPtCloudPtr);
    }
}

void PointCloudCapture::getImage(cv::Mat& img){
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr srcCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    getFrame(srcCloud);
    /*cv::Mat tempImg;*/
    /*img.create(cv::Size(srcCloud->width,srcCloud->height), CV_8UC3);*/
    makeImageFromPointCloud(img, *srcCloud);
    //tempImg.copyTo(img);
    return;
}

int PointCloudCapture::makeImageFromPointCloud(cv::Mat&  image,
    const pcl::PointCloud<pcl::PointXYZRGBA>& cloud)
{
    image.create(cv::Size(cloud.width,cloud.height), CV_8UC3);
    assert(image.cols == (int) cloud.width);
    assert(image.rows == (int) cloud.height);

    // First iterate over the pointcloud
    for(unsigned int h = 0; h < cloud.height; h++){
        for(unsigned int w = 0; w < cloud.width; w++){
            image.at<cv::Vec3b>(h,w)[0] = (cloud.points[h*cloud.width + w]).b;
            image.at<cv::Vec3b>(h,w)[1] = (cloud.points[h*cloud.width + w]).g;
            image.at<cv::Vec3b>(h,w)[2] = (cloud.points[h*cloud.width + w]).r;
        }
    }
    //cv::imshow("debug" , image); //#debug
    return 0;
}

void PointCloudCapture::getPreviousCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
    pcl::copyPointCloud(*ptCloudPtr, *cloud);
}

fake3DCamera::fake3DCamera(std::string filename){
    //open file
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (filename, *data.srcCloud) == -1) { //* load the file	
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        //return (-1);
    }
    makeImageFromPointCloud(data.srcImg, *data.srcCloud);
}

void fake3DCamera::getFrame(InputData& _data) {
    _data = data;
}
