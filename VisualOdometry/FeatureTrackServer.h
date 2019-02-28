//
// Created by steve on 1/16/19.
//

#ifndef BASESLAM_FEATURE_TRACKING_SERVER_H
#define BASESLAM_FEATURE_TRACKING_SERVER_H

#include <gtest/gtest.h>

#include <iostream>
#include <fstream>
#include <map>
#include <vector>

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>


class FeatureTrackServer {
public:
	FeatureTrackServer();

	~FeatureTrackServer();

	/**
	 * @brief Add new frame for extraction features.
	 * @param _img
	 * @return
	 */
	bool addNewFrame(cv::Mat &_img);

	/**
	 * @brief set mask according to camera model and tracked points from previous frame.
	 * @return
	 */
	bool setMask();

	/**
	 * @brief reject error pairs based on RANSAC-based method to estimate fundemental matrix.
	 * @return
	 */
	bool rejectWithFRANSAC();

	bool undistortedPoints();

	/**
	 * @brief add points to forw_pts_.
	 * @return
	 */
	bool addPoints2forw() {
		if (n_pts_.size() > 0) {
			for (auto &p:n_pts_) {
				forw_pts_.push_back(p);
				ids_.push_back(-1);
				track_cnt_.push_back(1);
			}
			return true;
		} else {
//			std::cout << "some problem that n_pts_ is empty" << std::endl;
			return false;
		}
	}

	/**
	 * @brief detect whether the point pt in the forw_img_.
	 * @param pt
	 * @return
	 */
	bool isInImage(cv::Point2f &pt);

	/**
	 * @brief set camera parameter.
	 * @param cam_mat
	 * @param dist_coeff
	 * @return
	 */
	bool setCameraParameter(cv::Mat &cam_mat, cv::Mat &dist_coeff) {
		cam_mat.copyTo(cam_mat_);
		dist_coeff.copyTo(dist_coeff_);
		return true;
	}


//private:
	// auxiliary mat
	cv::Mat mask_;
	cv::Mat prev_img_, cur_img_, forw_img_;
	std::vector<cv::Mat> prev_img_pyr_, cur_img_pyr_, forw_img_pyr_;



	//camera parameters
	cv::Mat cam_mat_;
	cv::Mat dist_coeff_;

	/**
	 * @brief  hyper parameters
	 */
	int max_features_ = 200;
	int min_feature_dis_ = 15;// pixel

	int pyr_patch_size_ = 10;
	int pyr_levels_ = 5;


	bool debug_flag_ = true; // for debug
	std::ofstream out_file_stream_;// output file.
	/////////////////////////////////////////////


	std::vector<cv::Point2f> n_pts_;
	std::vector<cv::Point2f> cur_pts_, forw_pts_, pre_pts_, prev_un_pts_, cur_un_pts_;
	std::vector<int> ids_, track_cnt_;

	long curr_feature_id_ = 0; // offset
	int cur_frame_id_ = 0; // id of readed img

};

/**
 * @brief reduce some element from the vecotr<VecType>
 * @tparam VecType recommended type is int float and uchar.
 * @param v vector<VecType> input array .
 * @param status vector<uchar> mask array, size of which should equal to v.
 * The element which's mask == 0 is deleted from the input vector
 */
template<typename VecType>
void reduceVector(std::vector<VecType> &v, std::vector<uchar> status) {
	int j = 0;
	for (int i = 0; i < int(v.size()); ++i) {
		if (status[i]) {
			v[j++] = v[i];
		}
	}
	v.resize(j);
}


/**
 * @brief
 * @param img Gray image(CV_8UC1) only
 * @return
 */
float blur_evaluate(cv::Mat &img){
	cv::Mat img_laplacian;
	cv::Laplacian(img,img_laplacian,CV_8UC1);
	return cv::mean(img_laplacian)[0];

}

#endif //BASESLAM_FEATURE_TRACKING_SERVER_H
