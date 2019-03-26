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

class StereoConfigServer{
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


	/**
	 * @brief Parameters for Feature tracking.
	 */

	int pyr_patch_size = 5;
	int pyr_levels = 3;

	int max_features = 200;
	int min_feature_dis = 200;





	bool stereo_model_flag = true;


	/**
	 *  Function
	 *
	 */

	static StereoConfigServer* getInstance(){

		static std::once_flag oc;

		if(instance_ == nullptr){
			std::call_once(oc,[]{
				if(instance_==nullptr){
					instance_ = new StereoConfigServer();
				}
			});
		}
		return instance_;
	}

protected:

	StereoConfigServer(){}

	~StereoConfigServer(){}

	StereoConfigServer(const StereoConfigServer &){}

	StereoConfigServer &operator=(const StereoConfigServer&){}

	static StereoConfigServer *instance_ ;



};

StereoConfigServer *StereoConfigServer::instance_ = nullptr;


#endif //PRACTICEVISUALPOSITIONING_STEREOCONFIGSERVER_H
