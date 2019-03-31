//
// Created by steve on 3/29/19.
//


#include <iostream>
#include <fstream>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <util/GeoTools.h>

#include "AWF.h"

#include <VisualizeTools/SLAMVisualServer.h>

#include <opencv2/opencv.hpp>


#include <VOSimulator/CameraProject.h>

int main() {

	auto logger_ptr = AWF::AlgorithmLogger::getInstance();


	auto slam_visulizer = BaseSLAM::SLAMVisualServer("gt trace");

	std::string dir_name = "/home/steve/temp/";

//	AWF::FileReader sim_frame_file(dir_name + "sim_frame_kpts.csv"),
//			sim_rframe_file(dir_name + "sim_rframe_kpts.csv"),
	AWF::FileReader sim_pos_file(dir_name + "sim_pos.csv"),
			sim_qua_file(dir_name + "sim_qua.csv"),
			kpts_file(dir_name + "kp_points.csv");

	Eigen::MatrixXd sim_pos = sim_pos_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd sim_qua = sim_qua_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd kpts3 = kpts_file.extractDoulbeMatrix(",");

	std::cout << "pos size:" << sim_pos.rows() << "," << sim_pos.cols() << std::endl;
	std::cout << "qua size:" << sim_qua.rows() << "," << sim_qua.cols() << std::endl;
	std::cout << "kpts3 size:" << kpts3.rows() << "," << kpts3.cols() << std::endl;

//	cv::Mat cloud(3,kpts3.rows(),CV_32F);
	std::vector<cv::Point3d> cloud;

	for (int i = 0; i < kpts3.rows(); ++i) {
		std::cout << kpts3.block<1, 3>(i, 0) << std::endl;
		cloud.push_back(cv::Point3f(kpts3(i, 0),
		                            kpts3(i, 1),
		                            kpts3(i, 2)));
	}
	slam_visulizer.addNewCloud("gt_points", cloud);


	CameraProject left_cameraProject(300.0, 300.0, 720, 1280);
	CameraProject right_cameraProject(300.0,300.0, 720,1280);

	left_cameraProject.setBody2Cam(

			)


	for (int i = 0; i < sim_pos.rows(); ++i) {
		logger_ptr->addTrace3dEvent(
				"src_trace",
				"gt_trace",
				sim_pos.block<1, 3>(i, 0)
		);

		slam_visulizer.addOdometryNewPose(
				sim_pos.block<1, 3>(i, 0).transpose(),
				Eigen::Quaterniond(sim_qua(i, 0),
				                   sim_qua(i, 1),
				                   sim_qua(i, 2),
				                   sim_qua(i, 3)),
				"ground truch"
		);

//		Eigen::MatrixXd pt_3d(kpts3.rows(), kpts3.cols());
		Eigen::MatrixXd pts_cam(kpts3.rows(), kpts3.cols());
		pts_cam.setZero();
		left_cameraProject.projectToimage(
				Eigen::Quaterniond(sim_qua(i, 0), sim_qua(i, 1), sim_qua(i, 2), sim_qua(i, 3)),
				sim_pos.block<1, 3>(i, 0).transpose(),
				kpts3,
				pts_cam
		);

		cv::Mat f_mat(left_cameraProject.height_,
		              left_cameraProject.width_,
		              CV_8UC3,
		              cv::Scalar(0, 0, 0));
		std::cout << "i:" << i <<
		          "/" << sim_pos.rows() << std::endl;
		for (int r = 0; r < pts_cam.rows(); ++r) {

			if (pts_cam(r, 2) > 0.0) {
				cv::circle(
						f_mat,
						cv::Point2f(pts_cam(r, 0), pts_cam(r, 1)),
						5,
						cv::Scalar(0, 0, 250));
				cv::putText(f_mat,
				            std::to_string(r),
				            cv::Point2f(pts_cam(r, 0), pts_cam(r, 1)),
				            1,
				            1.0,
				            cv::Scalar(100, 100, 0));
			}

		}


		cv::imshow("test", f_mat);

		cv::waitKey(10);
//		usleep(10000);
	}


	logger_ptr->outputAllEvent(true);
	cv::waitKey();

	return 0;

}