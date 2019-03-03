//
// Created by steve on 11/29/18.
//

#ifndef BASESLAM_SLAMVISUALSERVER_H
#define BASESLAM_SLAMVISUALSERVER_H

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

namespace BaseSLAM {
	class SLAMVisualServer {
	public:
		std::string windows_name_ = "default";

		cv::viz::Viz3d windows_;//(windows_name_);
		cv::viz::WTrajectory* wTrajectory_ptr_=nullptr;//default trajectory.

		std::vector<cv::Affine3d> pose_vec_;

		SLAMVisualServer(std::string windows_name) {
			windows_name_ = windows_name;
			windows_ = cv::viz::Viz3d(windows_name_);
//			wTrajectory_ = cv::viz::WTrajectory(pose_vec_);
//			windows_.showWidget("",)
		}


		bool addNewPose(cv::Affine3d pose){
			pose_vec_.push_back(pose);
			if(wTrajectory_ptr_==nullptr){
				wTrajectory_ptr_ = new cv::viz::WTrajectory(pose_vec_);
				windows_.showWidget("trajectory",*wTrajectory_ptr_);
			}else{
				wTrajectory_ptr_->updatePose(pose);
			}
			return true;
		}




	};

}


#endif //BASESLAM_SLAMVISUALSERVER_H
