//
// Created by steve on 3/25/19.
//

#ifndef PRACTICEVISUALPOSITIONING_STEREOFEATURETRACKER_H
#define PRACTICEVISUALPOSITIONING_STEREOFEATURETRACKER_H

#include <opencv2/opencv.hpp>

#include <StereoVO/StereoConfigServer.h>

class StereoFeatureTracker {
public://TODO: change to protect

	StereoFeatureTracker(){}

	~StereoFeatureTracker(){}

	bool addNewFrame(cv::Mat &l_img, cv::Mat &r_img);

	bool addNewFrame(cv::Mat &l_mat) {
		return addNewFrame(l_mat, l_mat);
	}

	bool setMask();

	bool rejectWithRANSAC();

	bool undistortedPoints();

	bool addPoint2forw();

	bool isInimage(const cv::Point2f &points);

	bool trackStereoPoints();

	// Values...
	StereoConfigServer *config_ptr_ = StereoConfigServer::getInstance();

	cv::Mat mask_; //  used in feature detection

	cv::Mat prev_img_, cur_img_, forw_img_; // image by main camera
	cv::Mat prev_r_img_, cur_r_img_, forw_r_img_;// image by right camera.

	std::vector<cv::Mat> prev_img_pyr_, cur_img_pyr_, forw_img_pyr_, forw_r_pyr_;// image pyr by main camera

	std::vector<cv::Point2f> n_pts_; // for saving detected new feature points

	std::vector<cv::Point2f> cur_pts_, forw_pts_, pre_pts_,
			prev_un_pts_, cur_un_pts_; // feature points for main camera
	std::vector<cv::Point2f> cur_r_pts, forw_r_pts, pre_r_pts_,
			prev_un_r_pts_, cur_un_r_pts_;

	std::vector<int> ids_, track_cnt_;
	std::vector<int> r_ids_, r_track_cnt_;

	int cur_frame_id_ = 0;
	int cur_feature_id = 0;


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

#endif //PRACTICEVISUALPOSITIONING_STEREOFEATURETRACKER_H
