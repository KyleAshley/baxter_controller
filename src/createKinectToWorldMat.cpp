#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <math.h>

#include "ros/ros.h"
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Geometry>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

using namespace std;
int main()
{

	/*
	double x = -0.5775705550653829;
	double y = 0.5676975541240944;
	double z = -0.4255263405695385;
	double w =  0.40380573849870377;
	*/
	double x = 0.1262533;
	double y = -0.34899919;
	double z = -0.67500178;
	double w = -0.63767724;
	
	double roll = 0.007059044266049735;
	double pitch = -0.31741403971898124;
	double yaw = 0.03086744338266941;

	tf::Vector3 origin;
	origin.setValue(0.15,0.08,0.56);

	tf::Matrix3x3 tf3d;
  	tf3d.setValue(0.00102448, -0.413635,  0.910442,
	-0.998378,  0.051398, 0.0244747,
	-0.0569185, -0.908991, -0.412912
	);

	//tf3d.getEulerYPR(tf3d);

	tf::Transform tf_trans; 
	tf::Quaternion tfqt;
	tf::Matrix3x3 r;

	r.setEulerYPR(  0.007059044266049735,-0.31741403971898124, 0.03086744338266941 );
  	//r.setEulerYPR(1.76857423019, -18.1864848341, 0.404453443841);
  	cout << "ROT " << r[0][0] << r[0][1] << r[0][2] << endl <<
  	 				  r[1][0] << r[1][1] << r[1][2] << endl <<
  	 				  r[2][0] << r[2][1] << r[2][2] << endl;
  	//tf3d.getRotation(tfqt);
  	cout <<"QUAT " << tfqt[0] << tfqt[1] << tfqt[2] << tfqt[3] << endl; 

  	tf_trans.setOrigin(origin);
	tf_trans.setRotation(tfqt);

	/*
	tf::Transform tf_trans; 
	tf::Quaternion tf_q(x, y, z, w);
	tf_trans.setOrigin(origin);
	tf_trans.setRotation(tf_q);
	*/

	Eigen::Matrix4f e_mat4;
	pcl_ros::transformAsMatrix(tf_trans, e_mat4);
	cout << e_mat4 << endl;
	cout << "------------------" << endl;
	/*******************************************/

	Eigen::Affine3d e_rot;
	tf::transformTFToEigen(tf_trans,e_rot);
	cout << e_rot.matrix() << endl;
	cout << "--------------" << endl;
	/*******************************************/

	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

	Eigen::Quaternion<double> q_idk = rollAngle * yawAngle * pitchAngle;

	Eigen::Matrix3d rotationMatrix = q_idk.matrix();
	cout << rotationMatrix << endl;
	cout << "--------------" << endl;
	/*******************************************/

	double cosRoll = cos(roll);
	double sinRoll = sin(roll);

	double cosPitch = cos(pitch);
	double sinPitch = sin(pitch);

	double cosYaw = cos(yaw);
	double sinYaw = sin(yaw);

	tf::Quaternion q(x, y, z, w);
	//q.setRPY(roll, pitch, yaw);
	Eigen::Quaterniond qe_test;
	Eigen::Quaterniond qe(w, x, y, z);

	tf::quaternionTFToEigen(q, qe_test);
	//cout << qe << endl;

	/*******************************************/
	Eigen::Quaternionf rotation(cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw, // represents w
                                sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,                   // represents x
                                cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,                 // represents y
                                cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw );                // represents z

	Eigen::Quaternionf rotation1(qe_test);
	Eigen::Matrix3f rotMatrix1 = rotation1.toRotationMatrix();
	std::cout << rotMatrix1 << std::endl;
	cout << "--------------" << endl;

	Eigen::Quaternionf rotation2(qe);
	Eigen::Matrix3f rotMatrix2 = rotation2.toRotationMatrix();
	std::cout << rotMatrix2 << std::endl;
	cout << "--------------" << endl;

	Eigen::Matrix3f rotMatrix3 = rotation.toRotationMatrix();
	std::cout << rotMatrix3 << std::endl;
	cout << "--------------" << endl;
}