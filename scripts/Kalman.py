

def initKalman(KF, cam_pos, rvecs, dt):

	KF.init(nStates, nMeasurements, nInputs, CV_64F); 					# init Kalman Filter
	cv.setIdentity(KF.processNoiseCov, cv.Scalar::all(1e-5)); 		# set process noise
	cv.setIdentity(KF.measurementNoiseCov, cv.Scalar::all(1e-4)); 	# set measurement noise
	cv.setIdentity(KF.errorCovPost, cv.Scalar::all(1)); 				# error covariance

	'''
	/* DYNAMIC MODEL */
	// [1 0 0 dt 0 0 dt2 0 0 0 0 0 0 0 0 0 0 0]
	// [0 1 0 0 dt 0 0 dt2 0 0 0 0 0 0 0 0 0 0]
	// [0 0 1 0 0 dt 0 0 dt2 0 0 0 0 0 0 0 0 0]
	// [0 0 0 1 0 0 dt 0 0 0 0 0 0 0 0 0 0 0]
	// [0 0 0 0 1 0 0 dt 0 0 0 0 0 0 0 0 0 0]
	// [0 0 0 0 0 1 0 0 dt 0 0 0 0 0 0 0 0 0]
	// [0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0]
	// [0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0]
	// [0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0]
	// [0 0 0 0 0 0 0 0 0 1 0 0 dt 0 0 dt2 0 0]
	// [0 0 0 0 0 0 0 0 0 0 1 0 0 dt 0 0 dt2 0]
	// [0 0 0 0 0 0 0 0 0 0 0 1 0 0 dt 0 0 dt2]
	// [0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 dt 0 0]
	// [0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 dt 0]
	// [0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 dt]
	// [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0]
	// [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0]
	// [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1]
	'''

	KF.transition_matrix[0][3] = dt;
	KF.transition_matrix[1][4] = dt;
	KF.transition_matrix[2][5] = dt;
	KF.transition_matrix[3][6] = dt;
	KF.transition_matrix[4][7] = dt;
	KF.transition_matrix[5][8] = dt;
	KF.transition_matrix[0][6] = 0.5*pow(dt,2);
	KF.transition_matrix[1][7] = 0.5*pow(dt,2);
	KF.transition_matrix[2][8] = 0.5*pow(dt,2);
	# orientation
	KF.transition_matrix[9][12] = dt;
	KF.transition_matrix[10][13] = dt;
	KF.transition_matrix[11][14] = dt;
	KF.transition_matrix[12][15] = dt;
	KF.transition_matrix[13][16] = dt;
	KF.transition_matrix[14][17] = dt;
	KF.transition_matrix[9][15] = 0.5*pow(dt,2);
	KF.transition_matrix[10][16] = 0.5*pow(dt,2);
	KF.transition_matrix[11][17] = 0.5*pow(dt,2);


	KF.measurement_matrix[0][0] = cam_pos[2]; 			# x
	KF.measurement_matrix[1][1] = -cam_pos[1]; 			# y
	KF.measurement_matrix[2][2] = cam_pos[0]; 			# z
	KF.measurement_matrix[3][9] = (rvecs[2])[0]; 		# roll
	KF.measurement_matrix[4][10] = -(rvecs[1])[0]; 		# pitch
	KF.measurement_matrix[5][11] = (rvecs[0])[0]); 		# yaw

def fillMeasurements( measurements, translation_measured, rotation_measured):

	# Convert rotation matrix to euler angles
	cv::Mat measured_eulers(3, 1, CV_64F);
	measured_eulers = rot2euler(rotation_measured);
	# Set measurement to predict
	measurements.at<double>(0) = translation_measured.at<double>(0); # x
	measurements.at<double>(1) = translation_measured.at<double>(1); # y
	measurements.at<double>(2) = translation_measured.at<double>(2); # z
	measurements.at<double>(3) = measured_eulers.at<double>(0); # roll
	measurements.at<double>(4) = measured_eulers.at<double>(1); # pitch
	measurements.at<double>(5) = measured_eulers.at<double>(2); # yaw



void updateKalmanFilter( cv::KalmanFilter &KF, cv::Mat &measurement,
cv::Mat &translation_estimated, cv::Mat &rotation_estimated )
{
// First predict, to update the internal statePre variable
cv::Mat prediction = KF.predict();
// The "correct" phase that is going to use the predicted value and our measurement
cv::Mat estimated = KF.correct(measurement);
// Estimated translation
translation_estimated.at<double>(0) = estimated.at<double>(0);
translation_estimated.at<double>(1) = estimated.at<double>(1);
translation_estimated.at<double>(2) = estimated.at<double>(2);
// Estimated euler angles
cv::Mat eulers_estimated(3, 1, CV_64F);
eulers_estimated.at<double>(0) = estimated.at<double>(9);
eulers_estimated.at<double>(1) = estimated.at<double>(10);
eulers_estimated.at<double>(2) = estimated.at<double>(11);
// Convert estimated quaternion to rotation matrix
rotation_estimated = euler2rot(eulers_estimated);
}
