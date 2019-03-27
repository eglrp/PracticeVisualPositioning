//
// Created by steve on 3/25/19.
//

#include "StereoFeatureManager.h"


bool StereoFeatureManager::addNewFrame(int frame_id,
                                       const std::vector<int> &feature_id_v,
                                       const std::vector<cv::Point2f> &feature_pts,
                                       const std::vector<int> &r_feature_id_v,
                                       const std::vector<cv::Point2f> &r_feature_pts) {
	// add frame to frame map
	FramePreId cur_frame(frame_id);
	if (config_ptr_->additional_check) {
		assert(feature_id_v.size() == feature_pts.size() &&
		       feature_id_v.size() == r_feature_id_v.size() &&
		       feature_id_v.size() == r_feature_pts.size());
	}

	for (int i(0); i < feature_id_v.size(); ++i) {

		// add feature to frame
		cur_frame.feature_id_vec_.push_back(feature_id_v[i]);
		cur_frame.id_pt_map.insert(std::make_pair(
				feature_id_v[i], feature_pts[i]
		));

		if (r_feature_id_v[i] == feature_id_v[i]) {
			cur_frame.id_r_pt_map.insert(
					std::make_pair(feature_id_v[i], r_feature_pts[i])
			);
		}

		// add feature
		auto itea = feature_map_.find(feature_id_v[i]);
		if (itea == feature_map_.end()) {
			// new feature  so added to feature map
			feature_map_.insert(std::make_pair(
					feature_id_v[i], FeaturePreId(feature_id_v[i], 1)
			));

			itea = feature_map_.find(feature_id_v[i]);

		}
		// add a record in  feature struct
		itea->second.frame_id_vec.push_back(frame_id);
		itea->second.tracked_counter += 1;
	}

	frame_map_.insert(std::pair(
			frame_id, cur_frame
	));



	// add new key frame.
	if (CheckKeyFrameCondition(cur_frame)) {
		return AddNewKeyFrame(cur_frame);

	} else {
		return true;
	}


}


bool StereoFeatureManager::CheckKeyFrameCondition(const FramePreId &cur_frame) {
	if (cur_frame.frame_id < 2) {
		return true;// add first two cur_frame. (Maybe just could be adopted in Stereo Visual Odometry)
	}

	// check co-vision features
	int co_cnt(0);
	double co_dis_sum(0.0);

	// JUST Process LAST FRAME
	auto last_key_frame_itea = frame_map_[key_frame_id_vec_[key_frame_id_vec_.size() - 1]];

	for (int i = 0; i < cur_frame.feature_id_vec_.size(); ++i) {
		int cur_feature_id = cur_frame.feature_id_vec_[i];
		auto itea = last_key_frame_itea.id_pt_map.find(cur_feature_id);
		if (itea != last_key_frame_itea.id_pt_map.end()) {
			co_cnt++;
			co_dis_sum += cv::norm(
					cur_frame.id_pt_map[cur_feature_id] - last_key_frame_itea.id_pt_map[cur_feature_id]
			);
		}

	}

	if (co_cnt < config_ptr_->min_covisible_feature) {
		return true;
	}

	if (co_dis_sum / double(co_cnt) > config_ptr_->min_parallex) {
		return true;
	}

	return false;
}

bool StereoFeatureManager::AddNewKeyFrame(FramePreId &cur_frame) {
	// record all new feature id in sw_feature_id_set_
	for(int i=0;i<cur_frame.feature_id_vec_.size();++i){
		int feature_id = cur_frame.feature_id_vec_[i];
		if(sw_feature_id_set_.find(feature_id) == sw_feature_id_set_.end()){
			sw_feature_id_set_.insert(feature_id);
			feature_map_[feature_id].in_slide_windows_flag=true;
		}
		feature_map_[feature_id].key_frame_id_deque.push_back(cur_frame.frame_id);// add
	}


	if(key_frame_id_vec_.size()<1){
		// initial
		cur_frame.initialized_pose = true;

	}else{
		// solve new pose by pnp
		std::vector<cv::Point3f> pts3;
		std::vector<cv::Point2f> ob_pt;
		for(int i=0;i<cur_frame.feature_id_vec_.size();++i){
			int cur_feature_id = cur_frame.feature_id_vec_[i];
			auto itea = sw_feature_id_set_.find(cur_feature_id);
			if(itea!=sw_feature_id_set_.end()&& feature_map_[*itea].initialized){
				auto pt_eigen = feature_map_[*itea].pt;
				pts3.push_back(cv::Point3f(pt_eigen[0],pt_eigen[1],pt_eigen[2]));
				ob_pt.push_back(cur_frame.id_pt_map[i]);
			}
		}

		cur_frame.qua = frame_map_[key_frame_id_vec_[key_frame_id_vec_.size()-1]].qua;
		cur_frame.pos = frame_map_[key_frame_id_vec_[key_frame_id_vec_.size()-1]].pos;

		if(solvePosePnp(cur_frame.qua,cur_frame.pos,
				ob_pt,pts3,config_ptr_->left_cam_mat,config_ptr_->left_dist_coeff)){

			cur_frame.initialized_pose = true;


		}else{
			printf("Some error when trying to calculate pnp\n");
			return false;

		}

	}

	// initial feature points by stereo observed.
	// TODO: Check the transfrom.
	Eigen::Matrix3d left_R = cur_frame.qua.toRotationMatrix() * config_ptr_->left_bodyTocam.block(0,0,3,3);
	Eigen::Matrix3d right_R = cur_frame.qua.toRotationMatrix() * config_ptr_->right_bodyTocam.block(0,0,3,3);
	Eigen::Vector3d left_t = config_ptr_->left_bodyTocam.block(0,0,3,3) * cur_frame.pos +
			config_ptr_->left_bodyTocam.block(0,3,3,1);
	Eigen::Vector3d right_t = config_ptr_->right_bodyTocam.block(0,0,3,3) * cur_frame.pos +
			config_ptr_->right_bodyTocam.block(0,3,3,1);

	for(int i=0;i<cur_frame.feature_id_vec_.size();++i){
		// feature not been initialized. and could be observed in

	}



	// initial feature points by two frame observed.



	// optimization
	Optimization();



	// update visulization.
	UpdateVisualization();


	// delete oldest frame in key frame slide windows.



	return true;
}

