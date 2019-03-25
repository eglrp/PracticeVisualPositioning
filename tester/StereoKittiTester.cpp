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


int main(){

	std::string dir_name = "/home/steve/SourceData/Kittidataset/sequences/05/";
	std::string left_sub_dir_name = "image_0";
	std::string right_sub_dir_name = "image_1";

	std::fstream list_file_stream(dir_name+left_sub_dir_name+".list");

	StereoConfigServer* config_ptr = StereoConfigServer::getInstance();

	config_ptr->stereo_model_flag = true;



	cv::Mat cam_mat(3, 3, CV_32F);
	cv::Mat coeff_mat(5, 1, CV_32F);

	cam_mat.at<float>(0, 0) = float(7.070912000000e+02);//fx
	cam_mat.at<float>(1, 1) = float(7.070912000000e+02);//fy
	cam_mat.at<float>(0, 2) = float(6.018873000000e+02);//cx
	cam_mat.at<float>(1, 2) = float(1.831104000000e+02);//cy
	cam_mat.at<float>(2, 2) = float(1.0);

	coeff_mat.at<float>(0, 0) = 0.0;//-0.467;
	coeff_mat.at<float>(1, 0) = 0.0;//0.168
	coeff_mat.at<float>(2, 0) = 0.0;
	coeff_mat.at<float>(3, 0) = 0.0;
	coeff_mat.at<float>(4, 0) = 0.0;

	cv::Mat btc_mat(4,4,CV_32F);

	for(int i=0;i<4;++i){
		btc_mat.at<float>(i,i) =1.0f;
	}

	btc_mat.at<float>(0,3) = 0.53715065326792f;

	std::string img_name;
	while(list_file_stream >> img_name){
		cv::Mat left_img = cv::imread(dir_name+left_sub_dir_name+"/"+img_name,
				cv::IMREAD_GRAYSCALE);
		cv::Mat right_img = cv::imread(dir_name+right_sub_dir_name+"/"+img_name,
				cv::IMREAD_GRAYSCALE);

		cv::imshow("left", left_img);
		cv::imshow("right",right_img);




		cv::waitKey(10);
	}


}
