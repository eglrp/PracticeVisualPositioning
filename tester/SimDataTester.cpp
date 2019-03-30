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

int main() {

	auto logger_ptr = AWF::AlgorithmLogger::getInstance();


	auto slam_visulizer = BaseSLAM::SLAMVisualServer("gt trace");

	std::string dir_name = "/home/steve/temp/";

//	AWF::FileReader sim_frame_file(dir_name + "sim_frame_kpts.csv"),
//			sim_rframe_file(dir_name + "sim_rframe_kpts.csv"),
	AWF::FileReader sim_pos_file(dir_name + "sim_pos.csv"),
			sim_qua_file(dir_name + "sim_qua.csv"),
			kpts_file(dir_name + "kp_points.csv");

//	Eigen::MatrixXd sim_frame = sim_frame_file.extractDoulbeMatrix(",");
//	Eigen::MatrixXd sim_rframe = sim_rframe_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd sim_pos = sim_pos_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd sim_qua = sim_qua_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd kpts3 = kpts_file.extractDoulbeMatrix(",");

//	std::cout << "frame size:" << sim_frame.rows() << " ," << sim_frame.cols() << std::endl;
//	std::cout << "rframe size:" << sim_rframe.rows() << "," << sim_rframe.cols() << std::endl;
	std::cout << "pos size:" << sim_pos.rows() << "," << sim_pos.cols() << std::endl;
	std::cout << "qua size:" << sim_qua.rows() << "," << sim_qua.cols() << std::endl;
	std::cout << "kpts3 size:" << kpts3.rows() << "," << kpts3.cols() << std::endl;


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
				"ground truch",
				false
		);

//		cv::waitKey(100);
		usleep(100000);
	}


	logger_ptr->outputAllEvent(true);
	cv::waitKey();

	return 0;

}