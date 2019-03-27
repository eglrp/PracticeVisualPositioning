//
// Created by steve on 3/25/19.
//

#ifndef PRACTICEVISUALPOSITIONING_STEREOFEATUREMANAGER_H
#define PRACTICEVISUALPOSITIONING_STEREOFEATUREMANAGER_H

#include <vector>
#include <map>
#include <deque>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <ceres/ceres.h>

#include <StereoVO/StereoConfigServer.h>

struct FramePreId {
	FramePreId(int id) : frame_id(id) {
	}

	long frame_id;
	bool key_frame_flag = false;

	Eigen::Vector3d pos;
	Eigen::Quaterniond qua;

	std::vector<int> feature_id_vec_;
	std::map<int, cv::Point2f> id_pt_map;
	std::map<int, cv::Point2f> id_r_pt_map;
};

struct FeaturePreId {
	FeaturePreId(int id, int cnt = 1) : feature_id(id),
	                                    tracked_counter(cnt) {

	}

	int feature_id;
	int tracked_counter;

	Eigen::Vector3d pt;
	// may not be using.
	double inv_depth;
	int depth_frame_id;

	std::map<int, cv::Point2f> frame_pt_map;
	std::vector<int> key_frame_id_vec;

};

class StereoFeatureManager {
public:
	StereoFeatureManager() {
		config_ptr_ = StereoConfigServer::getInstance();
	}

	/**
	 * @brief  add new feature represented frame.
	 * @param frame_id
	 * @param feature_id
	 * @param feature_pts
	 * @param r_feature_id
	 * @param r_feature_pts
	 * @return
	 */
	bool addNewFrame(int frame_id,
	                 std::vector<int> feature_id,
	                 std::vector<cv::Point2f> feature_pts,
	                 std::vector<int> r_feature_id,
	                 std::vector<cv::Point2f> r_feature_pts);




	StereoConfigServer *config_ptr_;

	int cur_feature_id = -1;
	int cur_frame_id = -1;

	std::map<int, FeaturePreId> feature_map_;// save all features observed in the whole process
	std::map<int, FramePreId> frame_map_;// save all frame and its observed feature's id in the whole process.

	std::deque<int> key_frame_id_vec_;// save id of current key frame.
	std::map<int, std::vector<int>> key_frame_feature_id_map_; // save frame_id -> feature id(contained in slide windows).


};


#endif //PRACTICEVISUALPOSITIONING_STEREOFEATUREMANAGER_H
