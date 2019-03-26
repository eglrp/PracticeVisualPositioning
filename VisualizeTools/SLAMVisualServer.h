//
// Created by steve on 11/29/18.
//

#ifndef BASESLAM_SLAMVISUALSERVER_H
#define BASESLAM_SLAMVISUALSERVER_H

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include <thread>

namespace BaseSLAM {
	class SLAMVisualServer {
	public:
		std::string windows_name_ = "default";

		cv::viz::Viz3d windows_;//(windows_name_);
//		cv::viz::WTrajectory *wTrajectory_ptr_ = nullptr;//default trajectory.

		std::vector<cv::Affine3d> pose_vec_;

		SLAMVisualServer(std::string windows_name) {
			windows_name_ = windows_name;
			windows_ = cv::viz::Viz3d(windows_name_);

		}



		/**
		 * @brief add new pose to odometry trajectory, which will not larger than 1000.
		 * @param pose
		 * @return
		 */
		bool addOdometryNewPose(cv::Affine3d pose) {
			pose_vec_.push_back(pose);
			windows_.showWidget("odometry_trajectory",
			                    cv::viz::WTrajectory(pose_vec_, 3));
			windows_.spinOnce();

			if (pose_vec_.size() > 1000) {
				pose_vec_.erase(pose_vec_.begin(), pose_vec_.begin() + 10);
			}
			return true;
		}


	};

}


#endif //BASESLAM_SLAMVISUALSERVER_H
