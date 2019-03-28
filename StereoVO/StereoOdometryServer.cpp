//
// Created by steve on 3/25/19.
//

#include "StereoOdometryServer.h"


bool StereoOdometryServer::addNewFrame(cv::Mat &left_img, cv::Mat &right_img) {

	cv::Mat pre_l_img, pre_r_img;
	assert(!left_img.empty());

	if (config_ptr_->stereo_model_flag && right_img.empty()) {
		printf("Set STEREO MODE in config but given a empty right image\n");
		exit(0);
	}


	if (left_img.channels() > 1) {
		cv::cvtColor(
				left_img,
				pre_l_img,
				cv::COLOR_BGR2GRAY
		);


	}

	if (config_ptr_->stereo_model_flag && right_img.channels() > 1) {

		cv::cvtColor(
				right_img,
				pre_r_img,
				cv::COLOR_BGR2GRAY
		);

	}

	// constraint limited histgram Equilization.
	if (config_ptr_->use_clahe) {

		cv::Ptr<cv::CLAHE> clahe_ptr = cv::createCLAHE(
				config_ptr_->clahe_para,
				cv::Size(config_ptr_->clahe_patch_size,
				         config_ptr_->clahe_patch_size)
		);

		clahe_ptr->apply(pre_l_img, pre_l_img);

		if (config_ptr_->stereo_model_flag) {
			clahe_ptr->apply(pre_r_img, pre_r_img);
		}
	}


	tracker_ptr_->addNewFrame(left_img, right_img);

	feature_manager_ptr_->addNewFrame(
			tracker_ptr_->cur_frame_id_,
			tracker_ptr_->ids_,
			tracker_ptr_->forw_pts_,
			tracker_ptr_->r_ids_,
			tracker_ptr_->forw_r_pts
	);

	if (feature_manager_ptr_->pose_deque.size() > 0) {
		for (int i = 0; i < feature_manager_ptr_->pose_deque.size(); ++i) {
			Eigen::Matrix4d pose = feature_manager_ptr_->pose_deque[i];
			cv::Mat R, t;
			cv::eigen2cv(Eigen::Matrix3d(pose.block(0, 0, 3, 3)), R);
			cv::eigen2cv(Eigen::Vector3d(pose.block(0, 3, 3, 1)), t);
			cv::Affine3d affine_3d(R, t);
			visualizer_ptr_->addOdometryNewPose(affine_3d);
		}

		feature_manager_ptr_->pose_deque.clear();
	}


}