/*
 * Software License Agreement (BSD License)
 *
 *  Object Pose Estimation (OPE) - www.cse.usf.edu/kkduncan/ope
 *  Copyright (c) 2013, Kester Duncan
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
 *	\author	Kester Duncan
 */
#pragma once
#ifndef __POINTCLOUDCAPTURE_H__
#define __POINTCLOUDCAPTURE_H__

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/openni_grabber.h>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <string>

/**
 * \brief Captures XYZRGB point clouds from the Kinect
 * \note The coordinate system for the captured point clouds is as follows:
 * x-axis -> right, y-axis -> down, z-axis -> points into scene.
 *
 * Also, a PassThrough filter is applied to the captured point cloud so that the
 * target area is within a specified range which is adjustable
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
	~PointCloudCapture();

	static int makeImageFromPointCloud(cv::Mat&  image,
	pcl::PointCloud<pcl::PointXYZRGBA>&  cloud);

	/**
	 * \brief Captures an XYZRGBA point cloud from the Kinect and stores an XYZRGB cloud
	 * \param <ptCloud> - holds the captured PointXYZRGB point cloud
	 */

	void startCapture();
	void stopCapture();
	void getFrame(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptCloud, cv::Mat &img);
	void getFrame(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ptCloud);
	void getImage(cv::Mat &img);
	/*temporary function to get the previously pointcloud.
	* used because frame_obj just takes images. remove this function after reorganizing
	*/
	void getPreviousCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

};


#endif /* __POINTCLOUDCAPTURE_H__ */

