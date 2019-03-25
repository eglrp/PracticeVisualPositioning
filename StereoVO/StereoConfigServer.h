//
// Created by steve on 3/25/19.
//

#ifndef PRACTICEVISUALPOSITIONING_STEREOCONFIGSERVER_H
#define PRACTICEVISUALPOSITIONING_STEREOCONFIGSERVER_H

#include <thread>
#include <mutex>

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

class StereoConfigServer{
public:

	cv::Mat left_cam_mat;
	cv::Mat right_cam_mat;
	cv::Mat left_dist_coeff;
	cv::Mat right_dist_coeff;

	Eigen::Matrix4d left_camTbody;
	Eigen::Matrix4d right_camTbody;


	bool stereo_model_flag = true;

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

	static StereoConfigServer *instance_;



};


#endif //PRACTICEVISUALPOSITIONING_STEREOCONFIGSERVER_H
