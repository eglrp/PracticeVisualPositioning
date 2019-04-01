//
// Created by steve on 11/29/18.
//

#ifndef BASESLAM_SLAMVISUALSERVER_H
#define BASESLAM_SLAMVISUALSERVER_H

#include <iostream>
#include <stdlib.h>

#include <unistd.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>


#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include <opencv2/core/eigen.hpp>

#include <thread>
#include <mutex>

namespace BaseSLAM {
	class SLAMVisualServer {
	public:
		std::string windows_name_ = "default";

		cv::viz::Viz3d windows_;//(windows_name_);
//		cv::viz::WTrajectory *wTrajectory_ptr_ = nullptr;//default trajectory.

//		std::vector<cv::Affine3d> pose_vec_;

		std::mutex trace_mutex;
		std::map<std::string, bool> name_flag_;
		std::map<std::string, std::vector<cv::Affine3d>> named_trace_;

		std::map<std::string, bool> cloud_name_cloud_;
		std::map<std::string, std::vector<cv::Point3d>> named_cloud_;


		bool running_state = true;

		std::thread running_thread;

		SLAMVisualServer(std::string windows_name) {
			windows_name_ = windows_name;
			windows_ = cv::viz::Viz3d(windows_name_);

			running_thread = std::thread([&]() {
				while (running_state) {

					refreshDisplay();
//					sleep(1);
					usleep(3000);
				}
			});


		}

		~SLAMVisualServer() {
			running_state = false;
			running_thread.join();
		}


		/**
		 * Refresh display windows based data saved in named_trace
		 */
		bool refreshDisplay() {

			windows_.removeAllWidgets();

			if (named_trace_.size() > 0) {
				for (auto &itea:named_trace_) {
					if (name_flag_[itea.first] == true) {
						windows_.showWidget(
								itea.first,
								cv::viz::WTrajectory(itea.second, cv::viz::WTrajectory::BOTH)
						);
					}
				}

			}

			if (named_cloud_.size() > 0) {
				for (auto &itea:named_cloud_) {
					windows_.showWidget(
							itea.first,
							cv::viz::WCloud(itea.second)
					);
				}

			}

			if (named_cloud_.size() > 0 || named_trace_.size() > 0) {

				windows_.spinOnce();
			}


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

			return true;

		}

		/**
		 *
		 */
		bool addOdometryNewPose(const Eigen::Vector3d &t,
		                        const Eigen::Matrix3d &R,
		                        const std::string &trace_name = "odo") {
			cv::Mat cvR(3, 3, CV_64F), cvt(3, 1, CV_64F);
			for (int x(0); x < 3; ++x) {
				for (int y(0); y < 3; ++y) {
					cvR.at<double>(x, y) = R(x, y);
				}
				cvt.at<double>(x, 0) = t(x);
			}

			return addOdometryNewPose(cv::Affine3d(cvR, cvt), trace_name);


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
		                        const std::string &trace_name = "odo") {
			return addOdometryNewPose(t, qua.toRotationMatrix(), trace_name);
		}

		/**
		 * @brief  delete all pose in the named trace(saved in a vector).
		 * @param trace_name
		 * @return
		 */
		bool deleteOdometryPose(std::string trace_name) {
			auto itea = named_trace_.find(trace_name);
			if (itea == named_trace_.end()) {
				return false;
			} else {
				name_flag_.erase(trace_name);
				named_trace_.erase(trace_name);
				return true;
			}


		}

		/**
		 * @brief delete number of element in the named trace
		 * @param trace_name
		 * @param delete_num  >0: delete latest n elements, <0: delete first n elements.
		 * @param pop_front
		 * @return
		 */
		bool deleteOdometryPose(std::string trace_name,
		                        int delete_num) {
			auto itea = named_trace_.find(trace_name);
			if (itea == named_trace_.end()) {
				// the trace name not contained in named_trace
				return false;
			} else {
				std::string tmp_name = itea->first;
				int vec_size = itea->second.size();
				if ((itea->second.size()) > std::abs(delete_num)) {

					// delete all element
					printf("deleted all elements in %s because the number of delete_num[%d] is larger than"
					       "size of vector[%d].\n", tmp_name.c_str(), std::abs(delete_num), vec_size);
					return deleteOdometryPose(trace_name);
				} else {

					auto &trace_vec = itea->second;
					if (delete_num > 0) {
						try {
							for (int i = 0; i < delete_num; ++i) {
								trace_vec.pop_back();
							}
						} catch (std::exception &e) {
							std::cout << __FILE__ << ":"
							          << __LINE__ << ":" << e.what() << std::endl;
						}


					} else {
						try {

							trace_vec.erase(trace_vec.begin(), trace_vec.begin() - delete_num);
						} catch (std::exception &e) {
							std::cout << __FILE__ << ":"
							          << __LINE__ << ":" << e.what() << std::endl;
							return false;
						}
					}

					return true;

				}

			}

		}

		bool addNewCloud(std::string cloud_name,
		                 std::vector<cv::Point3d> cloud_mat) {
			named_cloud_.insert(
					std::make_pair(cloud_name, cloud_mat)
			);
			return true;
		}


	};

}


#endif //BASESLAM_SLAMVISUALSERVER_H
