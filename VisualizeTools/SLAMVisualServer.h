//
// Created by steve on 11/29/18.
//

#ifndef BASESLAM_SLAMVISUALSERVER_H
#define BASESLAM_SLAMVISUALSERVER_H

#include <iostream>

#include <unistd.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include <opencv2/core/eigen.hpp>

#include <thread>

namespace BaseSLAM {
	class SLAMVisualServer {
	public:
		std::string windows_name_ = "default";

		cv::viz::Viz3d windows_;//(windows_name_);
//		cv::viz::WTrajectory *wTrajectory_ptr_ = nullptr;//default trajectory.

//		std::vector<cv::Affine3d> pose_vec_;

		std::map<std::string, std::vector<cv::Affine3d>> named_trace_;
		std::map<std::string, bool> name_flag_;

		bool running_state = true;

		std::thread running_thread;

		SLAMVisualServer(std::string windows_name) {
			windows_name_ = windows_name;
			windows_ = cv::viz::Viz3d(windows_name_);

			running_thread = std::thread([&](){
				while(running_state){

					refreshDisplay();
					sleep(1);
				}
			});




		}

		~SLAMVisualServer(){
			running_state = false;
			running_thread.join();
		}


		/**
		 * Refresh display windows based data saved in named_trace
		 */
		bool refreshDisplay() {
//			if (named_trace_.size() < 1) {
//				printf("%s:%s:Size of named_trace is 0, maybe some error generated!\n",
//				       __FILE__, __LINE__);
//				return false;
//			}
			for (auto &itea:named_trace_) {
				if (name_flag_[itea.first] == true) {
					windows_.showWidget(
							itea.first,
							cv::viz::WTrajectory(itea.second, cv::viz::WTrajectory::BOTH)
					);
				}

			}
			windows_.spinOnce();
			return true;

		}


		/**
		 * @brief add new pose to odometry trajectory, which will not larger than 1000.
		 * @param pose
		 * @return
		 */
		bool addOdometryNewPose(cv::Affine3d pose, std::string trace_name = "odo", bool refresh = true) {

			auto itea = named_trace_.find(trace_name);
			if (itea == named_trace_.end()) {
				named_trace_.insert(
						std::make_pair(trace_name, std::vector<cv::Affine3d>())
				);

				name_flag_.insert(
						std::make_pair(trace_name, true)
				);
				itea = named_trace_.find(trace_name);
			}
			itea->second.push_back(pose);

			if (refresh) {

				return refreshDisplay();
			} else {
				return true;
			}
		}

		/**
		 *
		 */
		bool addOdometryNewPose(const Eigen::Vector3d &t,
		                        const Eigen::Matrix3d &R,
		                        const std::string &trace_name = "odo",
		                        bool refresh = true) {
			cv::Mat cvR(3, 3, CV_64F), cvt(3, 1, CV_64F);
			for (int x(0); x < 3; ++x) {
				for (int y(0); y < 3; ++y) {
					cvR.at<double>(x, y) = R(x, y);
				}
				cvt.at<double>(x, 0) = t(x);
			}

			return addOdometryNewPose(cv::Affine3d(cvR, cvt), trace_name, refresh);


		}


		/**
		 * @brief
		 * @param t
		 * @param qua
		 * @param trace_name
		 * @return
		 */
		bool addOdometryNewPose(const Eigen::Vector3d &t,
		                        const Eigen::Quaterniond &qua,
		                        const std::string &trace_name = "odo",
		                        bool refresh = true) {
			return addOdometryNewPose(t, qua.toRotationMatrix(), trace_name, refresh);


		}


	};

}


#endif //BASESLAM_SLAMVISUALSERVER_H
