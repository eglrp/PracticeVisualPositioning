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
	if (key_frame_id_vec_.size() < 2) {
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
	std::cout << "key frame:";
	for (auto &itea:key_frame_id_vec_) {
		std::cout << itea << ",";
	}
	std::cout << std::endl;
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
		cur_frame.pos = Eigen::Vector3d(0, 0, 0);
		cur_frame.qua = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

	} else {
		// solve new pose by pnp
		std::vector<cv::Point3f> pts3;
		std::vector<cv::Point2f> ob_pt;
		for (int i = 0; i < cur_frame.feature_id_vec_.size(); ++i) {
			int cur_feature_id = cur_frame.feature_id_vec_[i];
			auto itea = sw_feature_id_set_.find(cur_feature_id);
			if (itea != sw_feature_id_set_.end()) {
				auto feature_itea = feature_map_.find(cur_feature_id);
				if (config_ptr_->additional_check && feature_itea == feature_map_.end()) {
					printf("Some error in %s:%s : can not found feature in feature map\n",
					       __FUNCTION__, __LINE__);

				} else {
					FeaturePreId *feature_ptr = &(feature_itea->second);
					if (feature_ptr->initialized) {
						auto pt_eigen = feature_ptr->pt;
//						std::cout << "getted pt:" << pt_eigen.transpose() << std::endl;
						pts3.push_back(cv::Point3f(pt_eigen[0], pt_eigen[1], pt_eigen[2]));
						ob_pt.push_back(cv::Point2f(cur_frame.id_pt_map[feature_ptr->feature_id].x,
						                            cur_frame.id_pt_map[feature_ptr->feature_id].y));
					}
				}

			}
		}

		// initial cur quaternion and pos based on latest key frame's pose.
		// TODO:Maybe need performence improvement....
		cur_frame.qua = frame_map_.find(key_frame_id_vec_[key_frame_id_vec_.size() - 2])->second.qua.normalized();
		cur_frame.pos = frame_map_.find(key_frame_id_vec_[key_frame_id_vec_.size() - 2])->second.pos * 1.0;


		Eigen::Quaterniond q_bc(config_ptr_->left_bodyTocam.block<3, 3>(0, 0));
		Eigen::Vector3d t_bc(config_ptr_->left_bodyTocam.block<3, 1>(0, 3));

//		std::cout << "found 3d points number:" << pts3.size() << std::endl;

//		std::cout << "result after initial:" << cur_frame.pos.transpose()
//		          << "\n cur quaternion:" << cur_frame.qua.toRotationMatrix() << std::endl;

		if (solvePosePnpCeres(cur_frame.qua,
		                      cur_frame.pos,
		                      q_bc,
		                      t_bc,
		                      ob_pt,
		                      pts3,
		                      config_ptr_->left_cam_mat,
		                      config_ptr_->left_dist_coeff)) {
//			std::cout << "solved pnp and get position:" << cur_frame.pos.transpose()
//			          << "\n quat:" << cur_frame.qua.matrix()
//			          << " \nused points:" << ob_pt.size() << std::endl;
			cur_frame.initialized_pose = true;

		} else {
			printf("Some error when trying to calculate pnp\n");
			std::cout << "pnp used points number:" << ob_pt.size() << std::endl;
			return false;// return false or continue next step?
		}

	}

	// initial feature points by stereo observed.
	// TODO: Check the transfrom.
	Eigen::Matrix3d left_R =
			config_ptr_->left_bodyTocam.block<3, 3>(0, 0) * cur_frame.qua.inverse().toRotationMatrix();

	Eigen::Matrix3d right_R =
			config_ptr_->right_bodyTocam.block<3, 3>(0, 0) * cur_frame.qua.inverse().toRotationMatrix();

	Eigen::Vector3d left_t = left_R * cur_frame.pos * -1.0 + config_ptr_->left_bodyTocam.block<3, 1>(0, 3);

	Eigen::Vector3d right_t = right_R * cur_frame.pos * -1.0 + config_ptr_->right_bodyTocam.block<3, 1>(0, 3);

	for (int i = 0; i < cur_frame.feature_id_vec_.size(); ++i) {
		// feature not been initialized. and could be observed in stereo
		int cur_feature_id = cur_frame.feature_id_vec_[i];
		FeaturePreId *feature_ptr = &(feature_map_.find(cur_feature_id)->second);
//		std::cout << "stere left t:" << left_t.transpose() << std::endl;

		// in slide windows  && not initialized && observed by right camera
		if (feature_ptr->in_slide_windows_flag &&
		    !feature_ptr->initialized &&
		    cur_frame.id_r_pt_map.find(cur_feature_id) != cur_frame.id_r_pt_map.end()
				) {

			Eigen::Vector2d left_ob(cur_frame.id_pt_map[cur_feature_id].x, cur_frame.id_pt_map[cur_feature_id].y);
			Eigen::Vector2d right_ob(cur_frame.id_r_pt_map[cur_feature_id].x, cur_frame.id_r_pt_map[cur_feature_id].y);

			if ((left_ob - right_ob).norm() > config_ptr_->min_ob_distance) {
				Eigen::Vector3d out_pt3(cur_frame.pos);
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

				if ((pre_ob - cur_ob).norm() > config_ptr_->min_ob_distance &&
				    (pre_key_frame->pos - cur_frame.pos).norm() > 0.5) {

					Eigen::Matrix3d pre_R =
							config_ptr_->left_bodyTocam.block<3, 3>(0, 0) *
							pre_key_frame->qua.inverse().toRotationMatrix();
					Eigen::Vector3d pre_t =
							pre_R * (pre_key_frame->pos * -1.0) + config_ptr_->left_bodyTocam.block(0, 3, 3, 1);


					Eigen::Vector3d out_pt3(0, 0, 0);
					if (triangulatePointCeres(
									Eigen::Quaterniond(pre_R),
									pre_t,
									Eigen::Quaterniond(left_R),
									left_t,
									config_ptr_
											->left_cam_mat,
									pre_ob, cur_ob,
									out_pt3
							)) {
						feature_ptr->initialized = true;
						feature_ptr->pt = out_pt3 * 1.0;
						continue;
					}
//
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
		FramePreId &oldest_frame = (frame_map_.find(key_frame_id_vec_[0])->second);
		key_frame_id_vec_.pop_front();
		oldest_frame.key_frame_flag = false;



		/**
		 * FEATURE:
		 * 1. delete related feature's key_frame_id_deque.
		 * 2. if size of (key frame id deque) < 2:
		 * 		clear key frame id deque,
		 * 		set in slide windows flag = false;
		 * 		deleted from sw feature id set
		 */

		for (int i = 0; i < oldest_frame.feature_id_vec_.size(); ++i) {
			FeaturePreId *feature_ptr =
					&(feature_map_.find(oldest_frame.feature_id_vec_[i])->second);

			auto itea = std::find_if(feature_ptr->key_frame_id_set.begin(),
			                         feature_ptr->key_frame_id_set.end(),
			                         [&](int t_id) {
				                         return t_id == oldest_frame.frame_id;
			                         });

			if (feature_ptr->key_frame_id_set.size() < 1) {
				feature_ptr->in_slide_windows_flag = false;
				feature_ptr->key_frame_id_set.clear();
				sw_feature_id_set_.erase(feature_ptr->feature_id);
			}
		}

	}


	return true;
}

bool StereoFeatureManager::Optimization() {

	if (key_frame_id_vec_.size() > 5) {
		ceres::Problem problem;
		ceres::Solver::Options options;
		ceres::Solver::Summary summary;

		double qua_array[key_frame_id_vec_.size() * 4];
		double pos_array[key_frame_id_vec_.size() * 4];

		double fx(config_ptr_->left_cam_mat.at<float>(0, 0));
		double fy(config_ptr_->left_cam_mat.at<float>(1, 1));
		double cx(config_ptr_->left_cam_mat.at<float>(0, 2));
		double cy(config_ptr_->left_cam_mat.at<float>(1, 2));

		Eigen::Quaterniond left_q_bc(config_ptr_->left_bodyTocam.block<3,3>(0,0));
		Eigen::Quaterniond right_q_bc(config_ptr_->right_bodyTocam.block<3,3>(0,0));

		Eigen::Vector3d left_t_bc(config_ptr_->left_bodyTocam.block<3,1>(0,3));
		Eigen::Vector3d right_t_bc(config_ptr_->right_bodyTocam.block<3,1>(0,3));

		double left_q_bc_array[4];
		double right_q_bc_array[4];
		double left_t_bc_array[3];
		double right_t_bc_array[3];

		left_q_bc_array[0] = left_q_bc.w();
		left_q_bc_array[1] = left_q_bc.x();
		left_q_bc_array[2] = left_q_bc.y();
		left_q_bc_array[3] = left_q_bc.z();
		right_q_bc_array[0] = right_q_bc.w();
		right_q_bc_array[1] = right_q_bc.x();
		right_q_bc_array[2] = right_q_bc.y();
		right_q_bc_array[3] = right_q_bc.z();


		for(int i=0;i<3;++i){
			left_t_bc_array[i] = left_t_bc(i);
			right_t_bc_array[i] = right_t_bc(i);
		}




		std::map<int, double *> kp_map;

		for (int i = 0; i < key_frame_id_vec_.size(); ++i) {
			FramePreId &cur_frame = frame_map_.find(key_frame_id_vec_[i])->second;

			Eigen::Quaterniond q_inv = cur_frame.qua.inverse();

			qua_array[i * 4 + 0] = q_inv.w();
			qua_array[i * 4 + 1] = q_inv.x();
			qua_array[i * 4 + 2] = q_inv.y();
			qua_array[i * 4 + 3] = q_inv.z();

			pos_array[i * 3 + 0] = cur_frame.pos.x();
			pos_array[i * 3 + 1] = cur_frame.pos.y();
			pos_array[i * 3 + 2] = cur_frame.pos.z();

			problem.AddParameterBlock(qua_array + i * 4, 4, new ceres::QuaternionParameterization);
			problem.AddParameterBlock(pos_array + i * 3, 3);

			for (int j = 0; j < cur_frame.feature_id_vec_.size(); ++j) {
				int cur_feature_id = cur_frame.feature_id_vec_[j];

				if (sw_feature_id_set_.find(cur_feature_id) != sw_feature_id_set_.end() &&
				    feature_map_.find(cur_feature_id)->second.initialized == true) {

					if (kp_map.find(cur_feature_id) == kp_map.end()) {
						kp_map.insert(std::make_pair(
								cur_feature_id, new double[3]
						));
						double *pt_ptr = kp_map.find(cur_feature_id)->second;
						memcpy(pt_ptr,
						       feature_map_.find(cur_feature_id)->second.pt.data(),
						       3 * sizeof(double));
						problem.AddParameterBlock(pt_ptr, 3);
						if(feature_map_.find(cur_feature_id)->second.key_frame_id_set.size()>0.5 * config_ptr_->slide_windows_size){
							problem.SetParameterBlockConstant(pt_ptr);
						}
					}

					double *pt_ptr_read = kp_map.find(cur_feature_id)->second;

//					std::cout << "feature point" << cur_feature_id << " 3d:" << pt_ptr_read[0] << ","
//					          << pt_ptr_read[1] << ","
//					          << pt_ptr_read[2] << std::endl;

					auto left_itea = cur_frame.id_pt_map.find(cur_feature_id);
					if (left_itea != cur_frame.id_pt_map.end()) {
						problem.AddResidualBlock(
								SimpleReprojectionError::Create(
										fx, fy, cx, cy,
										double(left_itea->second.x),
										double(left_itea->second.y),
										left_q_bc_array,
										left_t_bc_array
								),
//								NULL,
								new ceres::CauchyLoss(4.0),
								pt_ptr_read,
								qua_array + i * 4,
								pos_array + i * 3
						);


					}
					auto right_itea = cur_frame.id_r_pt_map.find(cur_feature_id);

					if (right_itea != cur_frame.id_r_pt_map.end()) {
						problem.AddResidualBlock(
								SimpleReprojectionError::Create(
										fx, fy, cx, cy,
										double(right_itea->second.x),
										double(right_itea->second.y),
										right_q_bc_array,
										right_t_bc_array
								),
//								NULL,
								new ceres::CauchyLoss(4.0),
								pt_ptr_read,
								qua_array + i * 4,
								pos_array + i * 3
						);
					}


					if (i < 1) {
						problem.SetParameterBlockConstant(qua_array + i * 4);
						problem.SetParameterBlockConstant(pos_array + i * 3);
					}


				}

			}
		}

		options.linear_solver_type = ceres::DENSE_SCHUR;
//		options.trust_region_strategy_type = ceres::DOGLEG;
//		options.check_gradients = true;
		options.num_threads = 8;
		options.num_linear_solver_threads=8;

//		options.max_num_iterations = 200;


		ceres::Solve(options, &problem, &summary);
		std::cout << summary.FullReport() << std::endl;

		for (int i = 0; i < key_frame_id_vec_.size(); ++i) {
			FramePreId &cur_frame = frame_map_.find(key_frame_id_vec_[i])->second;
			cur_frame.pos = Eigen::Vector3d(pos_array[i * 3], pos_array[i * 3 + 1], pos_array[i * 3 + 2]);
			cur_frame.qua = Eigen::Quaterniond(qua_array[i * 4], qua_array[i * 4 + 1], qua_array[i * 4 + 2],
			                                   qua_array[i * 4 + 3]).inverse();
		}

		for (auto &itea:kp_map) {

			FeaturePreId &feature = feature_map_.find(itea.first)->second;
			feature.pt = Eigen::Vector3d(
					itea.second[0],
					itea.second[1],
					itea.second[2]
			);

			if(feature.initialized==false){
				feature.initialized=true;
			}
//			std::cout << "feature " << itea.first
//			          << ":" << itea.second[0] << "," << itea.second[1] << "," << itea.second[2] << std::endl;
			free(itea.second);
		}

	}


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

	transform.block<3, 3>(0, 0) = frame_ptr->qua.toRotationMatrix() * 1.0;
	transform.block<3, 1>(0, 3) = frame_ptr->pos * 1.0;

	pose_deque.push_back(transform);

}
