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
	Eigen::Matrix3d rot;

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

	Eigen::Vector3d pos;

	std::map<int, cv::Point2f> frame_pt_map;

};

class StereoFeatureManager {
public:
	StereoFeatureManager() {
		config_ptr_ = StereoConfigServer::getInstance();
	}

	StereoConfigServer *config_ptr_;

	int cur_feature_id = -1;
	int cur_frame_id = -1;

	std::map<int, FeaturePreId> feature_map_;
	std::map<int, FramePreId> frame_map_;


};


#endif //PRACTICEVISUALPOSITIONING_STEREOFEATUREMANAGER_H
