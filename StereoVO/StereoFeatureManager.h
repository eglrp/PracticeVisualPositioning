//
// Created by steve on 3/25/19.
//

#ifndef PRACTICEVISUALPOSITIONING_STEREOFEATUREMANAGER_H
#define PRACTICEVISUALPOSITIONING_STEREOFEATUREMANAGER_H

#include <vector>
#include <map>
#include <deque>
#include <set>
#include <list>


#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <CeresSolverTools/SimpleReprojectionError.h>
#include <CeresSolverTools/SimpleKPPriorError.h>

#include <CeresSolverTools/SimpleInvDepthReProjectionError.h>
#include <CeresSolverTools/SimpleStereoInvDepthReprojectionError.h>

#include <CeresSolverTools/InvDepthReProjectionError.h>

#include <StereoVO/StereoConfigServer.h>

#include <util/GeoTools.h>

struct FramePreId {
//public:
	FramePreId(int id) : frame_id(id) {
	}

	int frame_id;
	bool key_frame_flag = false;

	bool initialized_pose = false;
	Eigen::Vector3d pos = Eigen::Vector3d::Zero();
	Eigen::Quaterniond qua = Eigen::Quaterniond::Identity();

//	double *pos_array = new double[3];
//	double *qua_array = new double[4];

	std::vector<int> feature_id_vec_; // id of contained features
	std::map<int, cv::Point2f> id_pt_map; // id -> pt(observed by left camera)
	std::map<int, cv::Point2f> id_r_pt_map; // id->pt(observed by right camera)

	// frame constraint dataset.
//	double *jac_

};

struct FeaturePreId {
//public:
	FeaturePreId(int id, int cnt = 1) : feature_id(id),
	                                    tracked_counter(cnt) {

	}

	int feature_id;
	int tracked_counter; //

	bool initialized = false;
	Eigen::Vector3d pt = Eigen::Vector3d(1000.0, 1000.0, 1000.0); // position of feature point in world frame.

	// may not be using.
//	double inv_depth = 1.0 / 50.0;// inverse depth of current feature in the frame which id is depth_frame_id
	int depth_frame_id = -1; //
	double *inv_depth_array = new double[1]{1.0 / 50.0};

//	std::map<int, cv::Point2f> frame_pt_map;
	std::vector<int> frame_id_vec;

	bool in_slide_windows_flag = false;
	std::deque<int> key_frame_id_deque; // the frame id which are constained current feature.

	std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> prior_info; //

};

class StereoFeatureManager {
public:
	StereoFeatureManager() {
		config_ptr_ = StereoConfigServer::getInstance();
	}

	/**
	 * @brief  add new feature represented frame.
	 * @param frame_id frame id of current frame
	 * @param feature_id_v feature id vector
	 * @param feature_pts feature points(2d)
	 * @param r_feature_id_v feature id for right ( set -1 if not observed by right cam)
	 * @param r_feature_pts
	 * @return
	 */
	bool addNewFrame(int frame_id,
	                 const std::vector<int> &feature_id_v,
	                 const std::vector<cv::Point2f> &feature_pts,
	                 const std::vector<int> &r_feature_id_v,
	                 const std::vector<cv::Point2f> &r_feature_pts);


	/**
	 * @brief Check if it's needed to add a new key frame.
	 * @return
	 */
	bool CheckKeyFrameCondition(FramePreId &cur_frame);


	/**
	 * @brief Add a new key frame and calculate it's pose.
	 * @param frame_id
	 * @return
	 */
	bool AddNewKeyFrame(int frame_id);

	/**
	 * @brief Sliding windows filter based optimization for feautres and key-frames.
	 * using reprojection error that position of feature points represented by position in world frame.
	 * @return
	 */
	bool Optimization();

	/**
	 * @brief projection error use inverse depth and projection between two frame.
	 * @return
	 */
	bool OptimizationCoP();

	/**
	 * @brief add latest key frame's pose to a deque.
	 * @param frame_id
	 * @return
	 */
	bool UpdateVisualization(int frame_id);

	StereoConfigServer *config_ptr_;

	int cur_feature_id = -1;
	int cur_frame_id = -1;


	std::map<int, FeaturePreId> feature_map_;// save all features observed in the whole process
	std::map<int, FramePreId> frame_map_;// save all frame and its observed feature's id in the whole process.

	// record key frame and related features.
	std::deque<int> key_frame_id_vec_;// save id of current key frame.
	std::set<int> sw_feature_id_set_; // feature id contained in slide windows

	std::deque<Eigen::Matrix4d> pose_deque;

};


#endif //PRACTICEVISUALPOSITIONING_STEREOFEATUREMANAGER_H
