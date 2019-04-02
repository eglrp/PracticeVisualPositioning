//
// Created by steve on 3/25/19.
//

#include <iostream>
#include <fstream>

#include <ceres/ceres.h>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


#include <StereoVO/StereoConfigServer.h>

#include <StereoVO/StereoOdometryServer.h>


int main() {

	std::string dir_name = "/home/steve/SourceData/Kittidataset/sequences/00/";
	std::string left_sub_dir_name = "image_0";
	std::string right_sub_dir_name = "image_1";

	std::fstream list_file_stream(dir_name + left_sub_dir_name + ".list");

	StereoConfigServer *config_ptr = StereoConfigServer::getInstance();

	config_ptr->stereo_model_flag = true;

	cv::Mat cam_mat(3, 3, CV_32F);
	cv::Mat coeff_mat(5, 1, CV_32F);

	cam_mat.at<float>(0, 0) = float(7.070912000000e+02);//fx
	cam_mat.at<float>(1, 1) = float(7.070912000000e+02);//fy
	cam_mat.at<float>(0, 2) = float(6.018873000000e+02);//cx
	cam_mat.at<float>(1, 2) = float(1.831104000000e+02);//cy
//	cam_mat.at<float>(2, 2) = float(1.0);

	// KITTI 00-02 and 12-21
//	cam_mat.at<float>(0, 0) = float(7.188560000000e+02);//fx
//	cam_mat.at<float>(1, 1) = float(7.188560000000e+02);//fy
//	cam_mat.at<float>(0, 2) = float(6.071928000000e+02);//cx
//	cam_mat.at<float>(1, 2) = float(1.852157000000e+02);//cy
	cam_mat.at<float>(2, 2) = float(1.0);

	coeff_mat.at<float>(0, 0) = 0.0;
	coeff_mat.at<float>(1, 0) = 0.0;
	coeff_mat.at<float>(2, 0) = 0.0;
	coeff_mat.at<float>(3, 0) = 0.0;
	coeff_mat.at<float>(4, 0) = 0.0;


	cam_mat.copyTo(config_ptr->left_cam_mat);
	cam_mat.copyTo(config_ptr->right_cam_mat);

	coeff_mat.copyTo(config_ptr->left_dist_coeff);
	coeff_mat.copyTo(config_ptr->right_dist_coeff);

	config_ptr->left_bodyTocam = Eigen::Matrix4d::Identity();
	config_ptr->right_bodyTocam = Eigen::Matrix4d::Identity();
	config_ptr->right_bodyTocam(0, 3) = -0.53715065326792;//*100.0;

	config_ptr->min_ob_distance = 10.0;

	config_ptr->max_features = 300;
	config_ptr->min_feature_dis  = 20;

	config_ptr->use_pyramid=true;


	config_ptr->slide_windows_size = 20;

	config_ptr->feature_quality = 0.01;

	config_ptr->min_covisible_feature = 150;
	config_ptr->min_parallex = 30.0;



	StereoOdometryServer odometry;


	std::string img_name;
	while (list_file_stream >> img_name) {
		cv::Mat left_img = cv::imread(dir_name + left_sub_dir_name + "/" + img_name,
		                              cv::IMREAD_GRAYSCALE);
		cv::Mat right_img = cv::imread(dir_name + right_sub_dir_name + "/" + img_name,
		                               cv::IMREAD_GRAYSCALE);

//		cv::imshow("left", left_img);
//		cv::imshow("right", right_img);

		odometry.addNewFrame(left_img, right_img);


		cv::waitKey(1);
	}

	cv::waitKey();


}
