//
// Created by steve on 3/25/19.
//

#ifndef PRACTICEVISUALPOSITIONING_STEREOCONFIGSERVER_H
#define PRACTICEVISUALPOSITIONING_STEREOCONFIGSERVER_H

#include <thread>
#include <mutex>

#include <fstream>

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

class StereoConfigServer {
public:

	/**
	 * @brief Parameters in whole Visual odometry System.
	 */
	cv::Mat left_cam_mat;
	cv::Mat right_cam_mat;
	cv::Mat left_dist_coeff;
	cv::Mat right_dist_coeff;

	Eigen::Matrix4d left_camTbody;
	Eigen::Matrix4d right_camTbody;

	bool debug_flag = true;

	bool stereo_model_flag = true;

	/**
	 * @brief Image preprocess parameters.
	 */

	bool use_clahe = true;
	float clahe_para = 3.0;
	int clahe_patch_size = 21;


	/**
	 * @brief Parameters for Feature tracking.
	 */

	bool use_pyramid = true;
	int pyr_patch_size = 21;
	int pyr_levels = 3;

	int max_features = 200;
	int min_feature_dis = 20;
	double feature_quality = 0.01;

	int lk_patch_size = 21;
	int lk_itea_count = 30;
	float lk_eps = 0.01;


	bool use_lk_reverse = true;
	float reverse_dis_threshold = 0.5;


	bool show_mask = true;


	double ransac_confidence = 0.99;
	double ransac_error = 1.0;



	/**
	 *  Function
	 *
	 */

	static StereoConfigServer *getInstance() ;
//	{
//
//		static std::once_flag oc;
//
//		if (instance_ == nullptr) {
//			std::call_once(oc, [] {
//				if (instance_ == nullptr) {
//					instance_ = new StereoConfigServer();
//				}
//			});
//		}
//		return instance_;
//	}

protected:

	StereoConfigServer() {}

	~StereoConfigServer() {}

	// In order to avoid illegal construct.
	StereoConfigServer(const StereoConfigServer &) {}

	StereoConfigServer &operator=(const StereoConfigServer &) {}

	static StereoConfigServer *instance_ ;


};




#endif //PRACTICEVISUALPOSITIONING_STEREOCONFIGSERVER_H
