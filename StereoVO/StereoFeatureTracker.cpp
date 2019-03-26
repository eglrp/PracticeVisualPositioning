//
// Created by steve on 3/25/19.
//

#include "StereoFeatureTracker.h"


bool StereoFeatureTracker::addNewFrame(cv::Mat &l_img, cv::Mat &r_img) {
	//region calculate pyramid for current frame
	std::vector <cv::Mat> t_l_img_pyr, t_r_img_pyr;
	if (config_ptr_->use_pyramid) {
		cv::buildOpticalFlowPyramid(
				l_img,
				t_l_img_pyr,
				cv::Size(
						config_ptr_->pyr_patch_size,
						config_ptr_->pyr_patch_size
				),
				config_ptr_->pyr_levels
		);

		if (config_ptr_->stereo_model_flag & !r_img.empty()) {
			cv::buildOpticalFlowPyramid(
					r_img,
					t_r_img_pyr,
					cv::Size(
							config_ptr_->pyr_patch_size,
							config_ptr_->pyr_patch_size
					),
					config_ptr_->pyr_levels
			);


		}

	}
	// endregion


	// region Set or reset img and pyramid.
	if (forw_img_.empty()) {
		// initial at such step.
		prev_img_ = cur_img_ = forw_img_ = l_img;

		if (config_ptr_->use_pyramid) {
			prev_img_pyr_.assign(t_l_img_pyr.begin(), t_l_img_pyr.end());
			cur_img_pyr_.assign(t_l_img_pyr.begin(), t_l_img_pyr.end());
			forw_img_pyr_.assign(t_l_img_pyr.begin(), t_l_img_pyr.end());
		}


		if (config_ptr_->stereo_model_flag && !r_img.empty()) {

			prev_r_img_ = cur_r_img_ = forw_r_img_ = r_img;
			if (config_ptr_->use_pyramid) {

				forw_r_pyr_.assign(t_r_img_pyr.begin(), t_r_img_pyr.end());
			}
		}

	} else {
		forw_img_ = l_img;


		if (config_ptr_->use_pyramid) {
			forw_img_pyr_.assign(t_l_img_pyr.begin(), t_l_img_pyr.end());
		}

		if (config_ptr_->stereo_model_flag && !r_img.empty()) {
			forw_r_img_ = r_img;
			if (config_ptr_->use_pyramid) {
				forw_r_pyr_.assign(t_r_img_pyr.begin(), t_r_img_pyr.end());
			}
		}
	}
	// endregion

	forw_pts_.clear();

	//region track feature in left image.
	if (cur_pts_.size() > 0) {
		std::vector <uchar> status;
		std::vector<float> err;

		if (config_ptr_->use_pyramid) {
			cv::calcOpticalFlowPyrLK(
					cur_img_pyr_,
					forw_img_pyr_,
					cur_pts_,
					forw_pts_,
					status,
					err,
					cv::Size(config_ptr_->lk_patch_size, config_ptr_->lk_patch_size),
					config_ptr_->pyr_levels,
					cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
					                 config_ptr_->lk_itea_count,
					                 config_ptr_->lk_eps)
			);

		} else {
			cv::calcOpticalFlowPyrLK(
					cur_img_,
					forw_img_,
					cur_pts_,
					forw_pts_,
					status,
					err,
					cv::Size(config_ptr_->lk_patch_size, config_ptr_->lk_patch_size),
					config_ptr_->pyr_levels,
					cv::TermCriteria(
							cv::TermCriteria::COUNT +
							cv::TermCriteria::EPS,
							config_ptr_->lk_itea_count,
							config_ptr_->lk_eps
					)
			);
		}

		//delete feature point out of image.
		for (int i = 0; i < forw_pts_.size(); ++i) {
			if (status[i] && !isInimage(forw_pts_[i])) {
				status[i] = 0;
			}
		}


		if (config_ptr_->use_lk_reverse) {
			std::vector <uchar> reverse_status;
			std::vector <cv::Point2f> reverse_pts = pre_pts_;

			//region reverse track using LK
			if (config_ptr_->use_pyramid) {
				cv::calcOpticalFlowPyrLK(
						forw_img_pyr_,
						cur_img_pyr_,
						forw_pts_,
						reverse_pts,
						reverse_status,
						err,
						cv::Size(config_ptr_->lk_patch_size, config_ptr_->lk_patch_size),
						config_ptr_->pyr_levels,
						cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
						                 config_ptr_->lk_itea_count,
						                 config_ptr_->lk_eps)
				);

			} else {
				cv::calcOpticalFlowPyrLK(
						forw_img_,
						cur_img_,
						forw_pts_,
						reverse_pts,
						reverse_status,
						err,
						cv::Size(config_ptr_->lk_patch_size, config_ptr_->lk_patch_size),
						config_ptr_->pyr_levels,
						cv::TermCriteria(
								cv::TermCriteria::COUNT +
								cv::TermCriteria::EPS,
								config_ptr_->lk_patch_size,
								config_ptr_->lk_eps
						)
				);
			}
			// endregion

			for (int i = 0; i < status.size(); ++i) {
				if (status[i] &&
				    reverse_status[i] &&
				    isInimage(reverse_pts[i]) &&
				    cv::norm(cur_pts_[i] - reverse_pts[i]) < config_ptr_->reverse_dis_threshold) {

					status[i] = 1;
				} else {
					status[i] = 0;
				}
			}

		}

		reduceVector<cv::Point2f>(pre_pts_, status);
		reduceVector<cv::Point2f>(cur_pts_, status);
		reduceVector<cv::Point2f>(forw_pts_, status);
		reduceVector<cv::Point2f>(cur_un_pts_, status);
		reduceVector<int>(ids_, status);
		reduceVector<int>(track_cnt_, status);

	}
	//endregion


	for (auto &n:track_cnt_) {
		n++;
	}

	rejectWithRANSAC();

	setMask();

	int n_max_cnt = config_ptr_->max_features -
	                static_cast<int>(forw_pts_.size());
	if (n_max_cnt > 0) {
		if (mask_.rows == forw_img_.rows && mask_.cols == forw_img_.cols) {
			cv::goodFeaturesToTrack(
					forw_img_,
					n_pts_,
					n_max_cnt,
					config_ptr_->feature_quality,
					config_ptr_->min_feature_dis,
					mask_,
					3, false, 0.04
			);
		} else {
			printf("mask size[%d,%d] is not same to forw image size[%d,%d]\n",
			       mask_.rows, mask_.cols, forw_img_.rows, forw_img_.cols);
		}
	} else {
		n_pts_.clear();
	}

	addPoint2forw();

	if (config_ptr_->stereo_model_flag) {
		trackStereoPoints();
	}


	prev_img_ = cur_img_;
	pre_pts_ = cur_pts_;

	cur_img_ = forw_img_;
	cur_pts_ = forw_pts_;

	prev_un_pts_ = cur_un_pts_;

	prev_img_pyr_.swap(cur_img_pyr_);
	cur_img_pyr_.swap(forw_img_pyr_);


	undistortedPoints();
	cv::Mat col_mat;
	cv::cvtColor(forw_img_, col_mat, cv::COLOR_GRAY2BGR);
	for (int i = 0; i < forw_pts_.size(); ++i) {
		if (track_cnt_[i] > 1) {

			if (track_cnt_[i] > 5) {
//				cv::putText(col_mat, std::to_string(ids_[i]),
//				            forw_pts_[i],
//				            3,
//				            1.0,
//				            cv::Scalar(0, 0, 100));
				cv::circle(col_mat, forw_pts_[i], 4, cv::Scalar(0, 0, 250), 4);
			} else {

				cv::circle(col_mat, forw_pts_[i], 3, cv::Scalar(0, 100, 150), 4);
			}


		} else {
			cv::circle(col_mat, forw_pts_[i], 2, cv::Scalar(0, 250, 0));
		}
	}

	cv::imshow("feature img", col_mat);

	if (config_ptr_->stereo_model_flag) {
		cv::Mat r_col_mat;
		cv::cvtColor(forw_r_img_,
		             r_col_mat,
		             cv::COLOR_GRAY2BGR);

		for(int i=0;i<forw_r_pts.size();++i){
			if(r_track_cnt_[i] > 0){
				cv::circle(r_col_mat,forw_r_pts[i],3,cv::Scalar(0,0,100),4);
			}
		}

		cv::imshow("right stereo img", r_col_mat);

	}


	return true;

}


bool StereoFeatureTracker::isInimage(const cv::Point2f &points) {
	if (points.x < 0 ||
	    points.x > forw_img_.cols ||
	    points.y < 0 ||
	    points.y > forw_img_.rows) {
		return false;
	} else {
		return true;
	}

}


bool StereoFeatureTracker::rejectWithRANSAC() {

	if (!(config_ptr_->left_cam_mat.empty()) &&
	    !(config_ptr_->left_dist_coeff.empty())) {

		if (forw_pts_.size() >= 8) {
			std::vector <cv::Point2f> ucur_pts, ufor_pts;
			std::vector <uchar> mask_status;


			cv::undistortPoints(
					cur_pts_,
					ucur_pts,
					config_ptr_->left_cam_mat,
					config_ptr_->left_dist_coeff,
					cv::noArray(),
					config_ptr_->left_cam_mat
			);

			cv::undistortPoints(
					forw_pts_,
					ufor_pts,
					config_ptr_->left_cam_mat,
					config_ptr_->left_dist_coeff,
					cv::noArray(),
					config_ptr_->left_cam_mat
			);

			cv::Mat F =
					cv::findFundamentalMat(
							ucur_pts,
							ufor_pts,
							cv::FM_RANSAC,
							config_ptr_->ransac_error,
							config_ptr_->ransac_confidence,
							mask_status
					);
			reduceVector<cv::Point2f>(pre_pts_, mask_status);
			reduceVector<cv::Point2f>(cur_pts_, mask_status);
			reduceVector<cv::Point2f>(forw_pts_, mask_status);
			reduceVector<cv::Point2f>(cur_un_pts_, mask_status);
			reduceVector<int>(ids_, mask_status);
			reduceVector<int>(track_cnt_, mask_status);

		} else {
			printf("tracked points in forw_pts less than 8\n");
		}

	} else {
		printf("left camera mat(K) or distort coefficient may not given!\n");
	}


}


///

bool StereoFeatureTracker::undistortedPoints() {

}
////

bool StereoFeatureTracker::setMask() {
	mask_ = cv::Mat(forw_img_.rows, forw_img_.cols, CV_8UC1, cv::Scalar(255));

	std::vector < std::pair < int, std::pair < cv::Point2f, int >> > cnt_pts_id;
	for (uint i(0); i < forw_pts_.size(); ++i) {
		cnt_pts_id.push_back(std::make_pair(track_cnt_[i], std::make_pair(forw_pts_[i], ids_[i])));
	}

	std::sort(
			cnt_pts_id.begin(),
			cnt_pts_id.end(),
			[](const std::pair<int, std::pair<cv::Point2f, int>> &a,
			   const std::pair<int, std::pair<cv::Point2f, int>> &b) {
				return a.first > b.first;
			}
	);

	forw_pts_.clear();
	ids_.clear();
	track_cnt_.clear();

	for (auto &it:cnt_pts_id) {
		if (mask_.at<uchar>(it.second.first) == 255) {
			forw_pts_.push_back(it.second.first);
			ids_.push_back(it.second.second);
			track_cnt_.push_back(it.first);
			cv::circle(mask_, it.second.first,
			           config_ptr_->min_feature_dis,
			           0,
			           -1);
		}
	}

	if (config_ptr_->show_mask) {
		cv::imshow("Mask", mask_);
	}

}


bool StereoFeatureTracker::addPoint2forw() {
	if (n_pts_.size() > 0) {
		for (auto &p:n_pts_) {
			forw_pts_.push_back(p);
			ids_.push_back(cur_feature_id);
			cur_feature_id++;
			track_cnt_.push_back(1);
		}
		return true;
	} else {
		if (config_ptr_->debug_flag) {

			printf("not any new key points\n");
		}
		return false;
	}
}


bool StereoFeatureTracker::trackStereoPoints() {
	assert(!forw_r_img_.empty());

	std::vector <uchar> valid_status;
	std::vector <uchar> r_valid_states;

	std::vector <cv::Point2f> right_pts;
	std::vector<int> right_ids_;

	std::vector<float> error;
	if (config_ptr_->use_pyramid) {

		cv::calcOpticalFlowPyrLK(
				forw_img_pyr_,
				forw_r_pyr_,
				forw_pts_,
				right_pts,
				valid_status,
				error,
				cv::Size(config_ptr_->lk_patch_size,
				         config_ptr_->lk_patch_size),
				config_ptr_->pyr_levels,
				cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
				                 config_ptr_->lk_itea_count,
				                 config_ptr_->lk_eps)
		);

	} else {
		cv::calcOpticalFlowPyrLK(
				forw_img_,
				forw_r_img_,
				forw_pts_,
				right_pts,
				valid_status,
				error,
				cv::Size(config_ptr_->lk_patch_size,
				         config_ptr_->lk_patch_size),
				config_ptr_->pyr_levels,
				cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
				                 config_ptr_->lk_itea_count,
				                 config_ptr_->lk_eps)
		);
	}


	if (config_ptr_->steres_use_lk_reverse) {
		std::vector <cv::Point2f> reverse_lpts;
		if (config_ptr_->use_pyramid) {
			cv::calcOpticalFlowPyrLK(
					forw_r_pyr_,
					forw_img_pyr_,
					right_pts,
					reverse_lpts,
					r_valid_states,
					error,
					cv::Size(config_ptr_->lk_patch_size,
					         config_ptr_->lk_patch_size),
					config_ptr_->pyr_levels,
					cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
					                 config_ptr_->lk_itea_count,
					                 config_ptr_->lk_eps)
			);
		} else {
			cv::calcOpticalFlowPyrLK(
					forw_r_img_,
					forw_img_,
					right_pts,
					reverse_lpts,
					r_valid_states,
					error,
					cv::Size(config_ptr_->lk_patch_size,
					         config_ptr_->lk_patch_size),
					config_ptr_->pyr_levels,
					cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
					                 config_ptr_->lk_itea_count,
					                 config_ptr_->lk_eps)
			);
		}


		for (int i = 0; i < forw_pts_.size(); ++i) {
			if (valid_status[i] &&
			    r_valid_states[i] &&
			    isInimage(reverse_lpts[i]) &&
			    cv::norm(forw_pts_[i] - reverse_lpts[i]) < config_ptr_->stereo_reverse_dis_threshold
					) {
				valid_status[i] = 1;

			} else {
				valid_status[i] = 0;

			}
		}

	}

	forw_r_pts.clear();
	r_ids_.clear();
	r_track_cnt_.clear();
	for (int i = 0; i < forw_pts_.size(); ++i) {
		if (valid_status[i] && isInimage(right_pts[i])) {
			// valid point
			r_ids_.push_back(ids_[i]);
			r_track_cnt_.push_back(track_cnt_[i]);

		} else {
			r_ids_.push_back(-1);
			r_track_cnt_.push_back(-1);

		}
		forw_r_pts.push_back(right_pts[i]);
	}


}

