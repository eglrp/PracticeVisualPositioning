//
// Created by steve on 1/16/19.
//

#include "FeatureTrackServer.h"


FeatureTrackServer::FeatureTrackServer() {
	out_file_stream_.open("/home/steve/temp/feature_track_server_out.txt");
	if (!out_file_stream_.is_open()) {
		std::cout << "out_file_stream_ not openned!" << std::endl;
	}
	out_file_stream_ << "BEGIN" << std::endl;


};

FeatureTrackServer::~FeatureTrackServer() {

	out_file_stream_ << "END" << std::endl;
};


bool FeatureTrackServer::addNewFrame(cv::Mat &_img) {
	cv::Mat img;//save img.

	cur_frame_id_++;// update frame id.


	// constraint limited histgram Equalization (easy to track feature).
	cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
	clahe->apply(_img, img);
	std::vector<cv::Mat> tmp_img_pyr;
	cv::buildOpticalFlowPyramid(
			img,
			tmp_img_pyr,
			cv::Size(pyr_patch_size_, pyr_patch_size_),
			pyr_levels_,
			true,
			cv::BorderTypes::BORDER_REFLECT101,
			cv::BorderTypes::BORDER_CONSTANT,
			false
	);


	double blur_score = blur_evaluate(img);


	if (forw_img_.empty()) {
		// initial feature tracking server.
		prev_img_ = cur_img_ = forw_img_ = img;

		prev_img_pyr_.assign(tmp_img_pyr.begin(), tmp_img_pyr.end());
		cur_img_pyr_.assign(tmp_img_pyr.begin(), tmp_img_pyr.end());
		forw_img_pyr_.assign(tmp_img_pyr.begin(), tmp_img_pyr.end());

	} else {
		forw_img_ = img;
		forw_img_pyr_.clear();
		forw_img_pyr_.assign(tmp_img_pyr.begin(), tmp_img_pyr.end());
	}

	forw_pts_.clear();

	if (cur_pts_.size() > 0) {
		std::vector<uchar> status;
		std::vector<float> err;

		//      change LK optical flow
//		cv::calcOpticalFlowPyrLK(cur_img_, forw_img_, cur_pts_, forw_pts_, status, err, cv::Size(21, 21), 3);
		cv::calcOpticalFlowPyrLK(cur_img_pyr_, forw_img_pyr_,
		                         cur_pts_, forw_pts_,
		                         status, err,
		                         cv::Size(pyr_patch_size_, pyr_patch_size_),
		                         pyr_levels_);

//		int un_valid_cnt = 0;
//		int long_term_cnt = 0;
//		for (int i = 0; i < status.size(); ++i) {
//			if (status[i] == 0) {
//				un_valid_cnt++;
//			}
//			if (track_cnt_[i] > 10) {
//				long_term_cnt++;
//			}
//		}
//		std::cout << "long_term_cnt:" << long_term_cnt
//		          << "un valid cnt:" << un_valid_cnt << "total point:" << status.size() << std::endl;


		for (int i = 0; i < int(forw_pts_.size()); ++i) {
			if (status[i] && !isInImage(forw_pts_[i])) {
				status[i] = 0;
			}

		}

		//Corrected
		reduceVector<cv::Point2f>(pre_pts_, status);
		reduceVector<cv::Point2f>(cur_pts_, status);
		reduceVector<cv::Point2f>(forw_pts_, status);
		reduceVector<cv::Point2f>(cur_un_pts_, status);
		reduceVector<int>(ids_, status);
		reduceVector<int>(track_cnt_, status);
	}

	//update counter for tracking.
	for (auto &n:track_cnt_) {
		n++;
	}

	// PUB_THIS_FRAME
	if (true) {
		rejectWithFRANSAC();
		setMask();

		//find new features
		/// TODO: try grid feature extraction.
		int n_max_cnt = max_features_ - static_cast<int>(forw_pts_.size());//??? the reason for using static cast
		if (n_max_cnt > 0) {
			if (mask_.empty()) {
				printf("mask is empty\n");
			}
			if (mask_.type() != CV_8UC1) {
				printf("mask type wrong\n");
			}
			if (mask_.size() != forw_img_.size()) {
				printf("wrong mask size");
			}

			if (mask_.rows == forw_img_.rows && mask_.cols == forw_img_.cols) {
				cv::goodFeaturesToTrack(forw_img_,
				                        n_pts_,
				                        max_features_ - forw_pts_.size(),
				                        0.01,
				                        min_feature_dis_,
				                        mask_);

			} else {
				std::cout << "mask_.size not same to forw_img_, previous is:" << mask_.size
				          << " last is:" << forw_img_.size << std::endl;
			}


		} else {
			n_pts_.clear();
		}

		//add points to
		addPoints2forw();


	}

	int lossed_cnt = 0;

	out_file_stream_ << "track_cnt:{";
	for (int i = 0; i < forw_pts_.size(); ++i) {
		if (track_cnt_[i] == 1) {
			lossed_cnt++;
		}


		out_file_stream_ << track_cnt_[i];
		if (i < forw_pts_.size() - 1) {
			out_file_stream_ << ",";

		}
	}
	out_file_stream_ << "}" << std::endl;

	out_file_stream_ << "blur_score:{"
	                 << blur_score << "}" << std::endl;

	// update here.

//	if(lossed_cnt<200|| cur_frame_id_<10){
	prev_img_ = cur_img_;
	pre_pts_ = cur_pts_;
	prev_un_pts_ = cur_un_pts_;
	cur_img_ = forw_img_;
	cur_pts_ = forw_pts_;

	prev_img_pyr_.swap(cur_img_pyr_);
	cur_img_pyr_.swap(forw_img_pyr_);

//	}

	undistortedPoints();


	// display
	cv::Mat col_mat;
	cv::cvtColor(forw_img_, col_mat, cv::COLOR_GRAY2BGR);
	for (int i = 0; i < forw_pts_.size(); ++i) {
		if (track_cnt_[i] > 1) {
//			cv::circle(col_mat, forw_pts_[i], (track_cnt_[i]), cv::Scalar(0, 200, 200));
			cv::circle(col_mat, forw_pts_[i], 3, cv::Scalar(250, 20, 20));
		} else {
			cv::circle(col_mat, forw_pts_[i], 1, cv::Scalar(200, 100, 0));
		}
	}
	cv::imshow("feature img", col_mat);

	return true;

}


bool FeatureTrackServer::isInImage(cv::Point2f &pt) {
	int width = forw_img_.cols;
	int height = forw_img_.rows;
	return pt.x > 0 && pt.x < width && pt.y > 0 && pt.y < height;
}


bool FeatureTrackServer::rejectWithFRANSAC() {
	if (cam_mat_.empty() && dist_coeff_.empty()) {
		std::cout << "cam_mat_ or dist_coeff_ with some problem(may be empty()" << std::endl;
	}
	if (forw_pts_.size() >= 8) {
		std::vector<cv::Point2f> un_cur_pts(cur_pts_.size()), un_forw_pts(forw_pts_.size());
		std::vector<uchar> mask_status;

		cv::undistortPoints(
				cur_pts_,
				un_cur_pts,
				cam_mat_,
				dist_coeff_,
				cv::noArray(),
				cam_mat_
		);

		cv::undistortPoints(
				forw_pts_,
				un_forw_pts,
				cam_mat_,
				dist_coeff_,
				cv::noArray(),
				cam_mat_
		);


		cv::findFundamentalMat(
				un_cur_pts,
				un_forw_pts,
				cv::FM_RANSAC,
				3,
				0.99,
				mask_status
		);


		cv::Mat essential_matrix =
				cv::findEssentialMat(
						un_cur_pts,
						un_forw_pts, cam_mat_
				);

		cv::Mat RR, TTt;
		try {

			cv::recoverPose(essential_matrix, un_cur_pts, un_forw_pts, cam_mat_, RR, TTt);
			std::cout << "RR:" << RR << "\nTTt:" << TTt << std::endl;

			RR.copyTo(R);
			TTt.copyTo(Tt);

		} catch (cv::Exception &e) {
			std::cout << e.what() << std::endl;
		}


		reduceVector<cv::Point2f>(pre_pts_, mask_status);
		reduceVector<cv::Point2f>(cur_pts_, mask_status);
		reduceVector<cv::Point2f>(forw_pts_, mask_status);
		reduceVector<cv::Point2f>(cur_un_pts_, mask_status);
		reduceVector<int>(ids_, mask_status);
		reduceVector<int>(track_cnt_, mask_status);

	} else {
		std::cout << "founded key points less than 8" << std::endl;
	}
}

bool FeatureTrackServer::setMask() {
	// set mask
	mask_ = cv::Mat(forw_img_.rows, forw_img_.cols, CV_8UC1, cv::Scalar(255));

	// prefer to keep features that tracked for long time.
	std::vector<std::pair<int, std::pair<cv::Point2f, int >>> cnt_pts_id;

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
			cv::circle(mask_, it.second.first, min_feature_dis_, 0, -1);

		}
	}

}

bool FeatureTrackServer::undistortedPoints() {

}