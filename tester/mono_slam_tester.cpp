//
// Created by steve on 3/3/19.
//

#include <opencv2/opencv.hpp>

#include <VisualOdometry/FeatureManager.h>
#include <VisualOdometry/FeatureManager.cpp>

#include <VisualOdometry/FeatureTrackServer.h>
#include <VisualOdometry/FeatureTrackServer.cpp>

#include <VisualOdometry/FrontEndOptimizer.h>
#include <VisualOdometry/FrontEndOptimizer.cpp>

#include <VisualOdometry/StereoCamera.h>

#include "util/MYNTEYEReader.h"

int main(){

	auto *stereo_camera_ptr = new BaseSLAM::StereoCamera("/home/steve/Data/MYNTVI/camera_parameter1.yaml");
	stereo_camera_ptr->print("camera");
//	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco0012.list");
//	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco009.list");
//	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco006.list");
	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco-hard1.list");
//	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco-hard2.list");

	cv::Mat whole_img, left_img,right_img;

	FeatureTrackServer featureTrackServer;
	featureTrackServer.setCameraParameter(stereo_camera_ptr->M1,stereo_camera_ptr->D1);
	for(int i=0;i<img_reader.vec_size_;++i){
		whole_img = img_reader.get_image(i);
		std::cout << " readed " << i << "-th image" << std::endl;


		left_img = img_reader.copy_left_img(whole_img);
		right_img = img_reader.copy_right_img(whole_img);

		cv::cvtColor(left_img,left_img,cv::COLOR_BGR2GRAY);

		if(i%3==0)
			featureTrackServer.addNewFrame(left_img);

		cv::imshow("left_src",left_img);
		cv::imshow("right_src", right_img);

		cv::waitKey(10);

	}

	cv::waitKey();

}
