/*
 * Software License Agreement (BSD License)
 *  
 * 
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *	\file	PointCloudCapture.h
 *	\author	Indika Pathirage
 */
#pragma once
#ifndef __POINTCLOUDCAPTURE_H__
#define __POINTCLOUDCAPTURE_H__

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/openni_grabber.h>
#include <opencv2/opencv.hpp>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <string>

#include "PointCloudCapture/InputData.h"

/**
 * \brief Captures XYZRGB point clouds from the Kinect
 */
class PointCloudCapture
{
private:
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptCloudPtr;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredPtCloudPtr;
    bool initialized;
    bool multipleKinect;
    std::string kinectID;

    bool needCloud;
    bool cloudDataReady;
    pcl::Grabber* kinInterface;


    /// \brief Callback function that is used to read XYZRGB point clouds from the Kinect
    void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	
public:
    PointCloudCapture();
    PointCloudCapture(std::string _kinectID);
    virtual ~PointCloudCapture();

    /**
     * \brief generates a 2D image from an ordered point cloud
     * \param <cloud> - an ordered point cloud of type pcl::PointXYZRGBA
     * \param <image> - output image passed as a reference
     */
    static int makeImageFromPointCloud(cv::Mat&  image,
                                       const pcl::PointCloud<pcl::PointXYZRGBA>&  cloud);

    /**
     * \brief Captures an XYZRGBA point cloud from the Kinect and stores an XYZRGB cloud
     * \param <ptCloud> - holds the captured PointXYZRGB point cloud
     */

    void startCapture();
    void stopCapture();
    void getFrame(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptCloud, cv::Mat &img);
    void getFrame(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptCloud);
    virtual void getFrame(InputData& data);
    void getImage(cv::Mat &img);
    /*temporary function to get the previously pointcloud.
    * used because frame_obj just takes images. remove this function after reorganizing
    */
    void getPreviousCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
};

class fake3DCamera: public PointCloudCapture{
public:
    fake3DCamera(std::string fileName);
    void getFrame(InputData& data);
private:
    InputData data;
};


#endif /* __POINTCLOUDCAPTURE_H__ */

