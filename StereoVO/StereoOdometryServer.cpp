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

	std::cout << "  frame id:       " << tracker_ptr_->cur_frame_id_
	          << "  feature num:    " << tracker_ptr_->ids_.size()
	          << "  max feature id :" << tracker_ptr_->ids_[tracker_ptr_->ids_.size() - 1] << std::endl;

	feature_manager_ptr_->addNewFrame(
			tracker_ptr_->cur_frame_id_,
			tracker_ptr_->ids_,
			tracker_ptr_->forw_pts_,
			tracker_ptr_->r_ids_,
			tracker_ptr_->forw_r_pts
	);

	if (feature_manager_ptr_->pose_deque.size() > 0) {
//		std::cout << "added new frame" << std::endl;
		for (int i = 0; i < feature_manager_ptr_->pose_deque.size(); ++i) {
			Eigen::Matrix4d pose = feature_manager_ptr_->pose_deque[i];

			trace_file << pose(0, 3) << "," << pose(1, 3) << "," << pose(2, 3) << ","
			           << "0.0,0.0,0.0" << std::endl;

			cv::Mat R(3,3,CV_64F), t(3,1,CV_64F);
//			cv::eigen2cv(Eigen::Matrix3d(pose.block(0, 0, 3, 3)), R);
//			cv::eigen2cv(Eigen::Vector3d(pose.block(0, 3, 3, 1)), t);
//			std::cout << "pose:\n" << t<< std::endl;
			for(int x(0);x<3;++x){
				for(int y(0);y<3;++y){
					R.at<double>(x,y) = (pose(x,y));
				}
				t.at<double>(x,0) = (pose(x,3));
			}

			std::cout << "pose:" << pose << std::endl;
			std::cout << "t:" << t << std::endl;

			cv::Affine3d affine_3d(R, t);
			visualizer_ptr_->addOdometryNewPose(affine_3d);
		}

		feature_manager_ptr_->pose_deque.clear();
	}


}