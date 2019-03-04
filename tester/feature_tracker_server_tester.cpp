//
// Created by steve on 1/16/19.
//
#include <iostream>

#include <opencv2/opencv.hpp>
#include <VisualOdometry/StereoCamera.h>


#include "VisualOdometry/FeatureTrackServer.h"
#include "VisualOdometry/FeatureTrackServer.cpp"


#include <opencv2/sfm.hpp>
#include <opencv2/sfm/reconstruct.hpp>
#include <opencv2/viz.hpp>
//#include <vector>

#include "util/MYNTEYEReader.h"

int main() {
	auto *stereo_camera_ptr = new BaseSLAM::StereoCamera("/home/steve/Data/MYNTVI/camera_parameter1.yaml");
	stereo_camera_ptr->print("camera");
//	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco0012.list");
//	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco006.list");
//	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco-hard1.list");
	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco-hard1.list");

	cv::Mat whole_img, left_img, right_img;

	FeatureTrackServer featureTrackServer;
	featureTrackServer.setCameraParameter(stereo_camera_ptr->M1, stereo_camera_ptr->D1);


	bool sfm_flag = false;
	std::vector<std::vector<cv::Point2f>> all_frame_pts;
	std::vector<std::vector<int>> all_frame_ids;
	int max_feature_id = 0;

//	if (sfm_flag) {

//	}

	for (int i = 0; i < img_reader.vec_size_; ++i) {
		whole_img = img_reader.get_image(i);
		std::cout << " readed " << i << "-th image" << std::endl;


		left_img = img_reader.copy_left_img(whole_img);
		right_img = img_reader.copy_right_img(whole_img);

		cv::cvtColor(left_img, left_img, cv::COLOR_BGR2GRAY);

		if (i % 3 == 0)
			featureTrackServer.addNewFrame(left_img);

//		if(sfm_flag){
//
//
//		}

		cv::imshow("left_src", left_img);
		cv::imshow("right_src", right_img);

		if (sfm_flag) {
			std::vector<cv::Point2f> cur_pts;
			std::vector<int> cur_ids;
			for (int fi = 0; fi < featureTrackServer.forw_pts_.size(); ++fi) {
				if (featureTrackServer.ids_[fi] > max_feature_id) {
					max_feature_id++;
				}
				cur_pts.push_back(featureTrackServer.forw_pts_[fi]);
				cur_ids.push_back(featureTrackServer.ids_[fi]);
			}
			all_frame_pts.push_back(cur_pts);
			all_frame_ids.push_back(cur_ids);
		} else {

			cv::waitKey(50);
		}

	}

	if (sfm_flag) {
		std::vector<std::vector<cv::Point2f>> points2d;
		for (int i = 0; i < all_frame_ids.size(); ++i) {
//			cv::sfm::reconstruction()
//			cv::sfm::importReconstruction()
//			cv::sfm::preconditionerFromPoints()
		}

	}

	cv::waitKey();


}