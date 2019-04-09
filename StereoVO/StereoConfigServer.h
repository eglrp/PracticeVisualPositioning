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

#include <ceres/ceres.h>

class StereoConfigServer {
public:

	/**
	 * @brief Parameters in whole Visual odometry System.
	 */
	cv::Mat left_cam_mat;
	cv::Mat right_cam_mat;
	cv::Mat left_dist_coeff;
	cv::Mat right_dist_coeff;

	Eigen::Matrix4d left_bodyTocam;
	Eigen::Matrix4d right_bodyTocam;

	bool debug_flag = true;


	bool additional_check = true; // additional check to avoid error understanding of coding.

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

	int max_features = 300;
	int min_feature_dis = 20;
	double feature_quality = 0.05;

	int lk_patch_size = 21;
	int lk_itea_count = 30;
	float lk_eps = 0.01;

	bool use_lk_reverse = true;
	float reverse_dis_threshold = 0.5;

	bool steres_use_lk_reverse = true;
	float stereo_reverse_dis_threshold = 0.5;

	bool show_mask = true;

	double ransac_confidence = 0.99;
	double ransac_error = 1.0;

	/**
	 * Parameters For Feature Manager
	 */
	int slide_windows_size = 20; //

	int min_covisible_feature = 100;// lower bound of tracked feature number.
	float min_parallex = 30.0; // average moving distance of all features.

	float min_ob_distance = 10.0;//

	/**
	 * Parameter For Optimizer
	 */
	int robust_kernel_type = 1;// 0:HuberLoss 1:CauchyLoss
	double robust_ratio = 1.0;// parameter for robust loss function.

	std::string solver_type = "DENSE_SCHUR";
	std::string trust_region_strategy_type = "DOGLEG";
	int max_num_itea = 10;
	double max_solver_time_in_seconds = 1.0;



	/**
	 *  Function
	 *
	 */

	static StereoConfigServer *getInstance() ;

protected:

	StereoConfigServer() {}

	~StereoConfigServer() {}

	// In order to avoid illegal construct.
	StereoConfigServer(const StereoConfigServer &) {}

	StereoConfigServer &operator=(const StereoConfigServer &) {}

	static StereoConfigServer *instance_ ;


};




#endif //PRACTICEVISUALPOSITIONING_STEREOCONFIGSERVER_H
