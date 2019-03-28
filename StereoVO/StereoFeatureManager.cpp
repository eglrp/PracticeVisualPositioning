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
		return AddNewKeyFrame(cur_frame.frame_id);

	} else {
		return true;
	}


}


bool StereoFeatureManager::CheckKeyFrameCondition(FramePreId &cur_frame) {
//	if (cur_frame.frame_id < 1 ){//|| key_frame_id_vec_.size() < 2) {
	if (key_frame_id_vec_.size() < 1) {
		return true;// add first two cur_frame. (Maybe just could be adopted in Stereo Visual Odometry)
	}

	// check co-vision features
	int co_cnt(0);
	double co_dis_sum(0.0);

	// JUST Process LAST FRAME
	auto last_key_frame_itea = frame_map_.find(key_frame_id_vec_[key_frame_id_vec_.size() - 1]);


	for (int i = 0; i < cur_frame.feature_id_vec_.size(); ++i) {
		int cur_feature_id = cur_frame.feature_id_vec_[i];
		auto itea = last_key_frame_itea->second.id_pt_map.find(cur_feature_id);
		if (itea != last_key_frame_itea->second.id_pt_map.end()) {
			co_cnt++;
			co_dis_sum += cv::norm(
					cur_frame.id_pt_map[cur_feature_id] - last_key_frame_itea->second.id_pt_map[cur_feature_id]
			);
		}

	}

	if (co_cnt < config_ptr_->min_covisible_feature) {
		std::cout << "co cnt:" << co_cnt << std::endl;
		return true;
	}

	if (co_dis_sum / double(co_cnt) > config_ptr_->min_parallex) {
		std::cout << "avg dis:" << co_dis_sum / double(co_cnt) << std::endl;
		return true;
	}

	return false;
}

bool StereoFeatureManager::AddNewKeyFrame(int frame_id) {
	key_frame_id_vec_.push_back(frame_id);
	FramePreId &cur_frame = frame_map_.find(frame_id)->second;

	//region record all new feature id in sw_feature_id_set_
	for (int i = 0; i < cur_frame.feature_id_vec_.size(); ++i) {
		int feature_id = cur_frame.feature_id_vec_[i];
		FeaturePreId *feature_ptr = &(feature_map_.find(feature_id)->second);
		if (sw_feature_id_set_.find(feature_id) == sw_feature_id_set_.end()) {
			sw_feature_id_set_.insert(feature_id);

			auto itea = feature_map_.find(feature_id);
			if (itea != feature_map_.end()) {
				itea->second.in_slide_windows_flag = true;
			}
		} else {
			if (config_ptr_->additional_check) {
				// if in slide windows flag is false,
				// the State saved here not same to the state in sw_feature_id_set_
				if (!(feature_ptr->in_slide_windows_flag)) {
					printf("Some Error that current frame set to in key frames");
				}
			}
		}
		feature_ptr->key_frame_id_set.push_back(cur_frame.frame_id);
	}
	//endregion


	// calculate initial pose of each new key frame.
	if (key_frame_id_vec_.size() < 2) {
		// initial
		cur_frame.initialized_pose = true;

	} else {
		// solve new pose by pnp
		std::vector<cv::Point3f> pts3;
		std::vector<cv::Point2f> ob_pt;
		for (int i = 0; i < cur_frame.feature_id_vec_.size(); ++i) {
			int cur_feature_id = cur_frame.feature_id_vec_[i];
			auto itea = sw_feature_id_set_.find(cur_feature_id);
			if (itea != sw_feature_id_set_.end()) {
				auto itea = feature_map_.find(cur_feature_id);
				if (config_ptr_->additional_check && itea == feature_map_.end()) {
					printf("Some error in %s:%s : can not found feature in feature map\n",
					       __FUNCTION__, __LINE__);

				} else {
					FeaturePreId *feature_ptr = &(itea->second);
					if (feature_ptr->initialized) {
						auto pt_eigen = feature_ptr->pt;
						pts3.push_back(cv::Point3f(pt_eigen[0], pt_eigen[1], pt_eigen[2]));
						ob_pt.push_back(cur_frame.id_pt_map[i]);
					}
				}

			}
		}

		// initial cur quaternion and pos based on latest key frame's pose.
		// TODO:Maybe need performence improvement....
		cur_frame.qua = frame_map_.find(key_frame_id_vec_[key_frame_id_vec_.size() - 1])->second.qua.normalized();
		cur_frame.pos = frame_map_.find(key_frame_id_vec_[key_frame_id_vec_.size() - 1])->second.pos * 1.0;

		if (solvePosePnp(cur_frame.qua,
		                 cur_frame.pos,
		                 ob_pt,
		                 pts3,
		                 config_ptr_->left_cam_mat,
		                 config_ptr_->left_dist_coeff)) {
			std::cout << "solved pnp and get position:" << cur_frame.pos
			          << " quat:" << cur_frame.qua.matrix() << " used points:" << ob_pt.size() << std::endl;
			cur_frame.initialized_pose = true;

		} else {
			printf("Some error when trying to calculate pnp\n");
			std::cout << "pnp used points number:" << ob_pt.size() << std::endl;
//			cur_frame.qua = frame_map_[key_frame_id_vec_[key_frame_id_vec_.size() - 1]].qua;
//			cur_frame.pos = frame_map_[key_frame_id_vec_[key_frame_id_vec_.size() - 1]].pos;
			return false;// return false or continue next step?
		}

	}

	// initial feature points by stereo observed.
	// TODO: Check the transfrom.
	Eigen::Matrix3d left_R = cur_frame.qua.toRotationMatrix() * config_ptr_->left_bodyTocam.block(0, 0, 3, 3);
	Eigen::Matrix3d right_R = cur_frame.qua.toRotationMatrix() * config_ptr_->right_bodyTocam.block(0, 0, 3, 3);
	Eigen::Vector3d left_t = config_ptr_->left_bodyTocam.block(0, 0, 3, 3) * cur_frame.pos +
	                         config_ptr_->left_bodyTocam.block(0, 3, 3, 1);
	Eigen::Vector3d right_t = config_ptr_->right_bodyTocam.block(0, 0, 3, 3) * cur_frame.pos +
	                          config_ptr_->right_bodyTocam.block(0, 3, 3, 1);

	for (int i = 0; i < cur_frame.feature_id_vec_.size(); ++i) {
		// feature not been initialized. and could be observed in stereo
		int cur_feature_id = cur_frame.feature_id_vec_[i];
		FeaturePreId *feature_ptr = &(feature_map_.find(cur_feature_id)->second);

		// in slide windows  && not initialized && observed by right camera
		if (feature_ptr->in_slide_windows_flag &&
		    !feature_ptr->initialized &&
		    cur_frame.id_r_pt_map.find(cur_feature_id) != cur_frame.id_r_pt_map.end()) {

			Eigen::Vector2d left_ob(cur_frame.id_pt_map[cur_feature_id].x, cur_frame.id_pt_map[cur_feature_id].y);
			Eigen::Vector2d right_ob(cur_frame.id_r_pt_map[cur_feature_id].x, cur_frame.id_r_pt_map[cur_feature_id].y);

			if ((left_ob - right_ob).norm() > config_ptr_->min_ob_distance) {
				Eigen::Vector3d out_pt3(0,0,0);

				if (triangulatePointCeres(
						Eigen::Quaterniond(left_R),
						left_t,
						Eigen::Quaterniond(right_R),
						right_t,
						config_ptr_->left_cam_mat,
						left_ob,
						right_ob,
						out_pt3
				)) {
					feature_ptr->initialized = true;
					feature_ptr->pt = out_pt3 * 1.0;
				}
			}


		}

		// initial feature points by two frame observed.
		if (feature_ptr->in_slide_windows_flag &&
		    !feature_ptr->initialized &&
		    feature_ptr->key_frame_id_set.size() > 1) {

			for (auto pre_key_id:feature_ptr->key_frame_id_set) {
				FramePreId *pre_key_frame = &(frame_map_.find(pre_key_id)->second);
				Eigen::Vector2d pre_ob(pre_key_frame->id_pt_map[cur_feature_id].x,
				                       pre_key_frame->id_pt_map[cur_feature_id].y);
				Eigen::Vector2d cur_ob(cur_frame.id_pt_map[cur_feature_id].x,
				                       cur_frame.id_pt_map[cur_feature_id].y);

				if ((pre_ob - cur_ob).norm() > config_ptr_->min_ob_distance) {
					Eigen::Matrix3d pre_R =
							pre_key_frame->qua.toRotationMatrix() * config_ptr_->left_bodyTocam.block(0, 0, 3, 3);
					Eigen::Vector3d pre_t = config_ptr_->left_bodyTocam.block(0, 0, 3, 3) * pre_key_frame->pos +
					                        config_ptr_->left_bodyTocam.block(0, 3, 3, 1);


					Eigen::Vector3d out_pt3(0,0,0);
					if (triangulatePointCeres(
							Eigen::Quaterniond(pre_R),
							pre_t,
							Eigen::Quaterniond(left_R),
							left_t,
							config_ptr_->left_cam_mat,
							pre_ob, cur_ob,
							out_pt3
					)) {
						feature_ptr->initialized = true;
						feature_ptr->pt = out_pt3 * 1.0;
						break;
					}

				}

			}

		}
	}


	// optimization
	Optimization();



	// update visulization.
	UpdateVisualization(cur_frame.frame_id);

	// delete oldest frame in key frame slide windows.

	if (key_frame_id_vec_.size() > config_ptr_->slide_windows_size) {
		/**
		 * FRAME:
		 * 1. set key frame flag = false
		 * 2. deleted from (key_frame_id_vec)
		 */
		FramePreId *oldest_frame_ptr = &(frame_map_.find(key_frame_id_vec_[0])->second);
		key_frame_id_vec_.pop_front();

		oldest_frame_ptr->key_frame_flag = false;



		/**
		 * FEATURE:
		 * 1. delete related feature's key_frame_id_deque.
		 * 2. if size of (key frame id deque) < 2:
		 * 		clear key frame id deque,
		 * 		set in slide windows flag = false;
		 * 		deleted from sw feature id set
		 */

		for (int i = 0; i < oldest_frame_ptr->feature_id_vec_.size(); ++i) {
			FeaturePreId *feature_ptr =
					&(feature_map_.find(oldest_frame_ptr->feature_id_vec_[i])->second);

			auto itea = std::find_if(feature_ptr->key_frame_id_set.begin(),
			                         feature_ptr->key_frame_id_set.end(),
			                         [&](int t_id) {
				                         return t_id == oldest_frame_ptr->frame_id;
			                         });

			if (feature_ptr->key_frame_id_set.size() < 2) {
				feature_ptr->in_slide_windows_flag = false;
				feature_ptr->key_frame_id_set.clear();
				sw_feature_id_set_.erase(feature_ptr->feature_id);
			}
		}

	}


	return true;
}

bool StereoFeatureManager::Optimization() {

}


bool StereoFeatureManager::UpdateVisualization(int frame_id) {

	Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
	if (frame_map_.find(frame_id) == frame_map_.end()) {
		std::cout << "EROOR FRAME ID:" << frame_id << std::endl;
	}
	FramePreId *frame_ptr = &(
			frame_map_.find(frame_id)->second
	);

	std::cout << "cur frame id[key frame]:" << frame_ptr->frame_id
	          << " is initialized:" << frame_ptr->initialized_pose << std::endl;

	transform.block(0, 0, 3, 3) = frame_ptr->qua.toRotationMatrix() * 1.0;
	transform.block(0, 3, 3, 1) = frame_ptr->pos * 1.0;

//	std::cout << " transfrom:\n"
//	          << transform << std::endl;
	pose_deque.push_back(transform.inverse());

}
