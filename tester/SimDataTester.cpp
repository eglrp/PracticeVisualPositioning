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

int main() {
	std::string dir_name = "/home/steve/temp/";

	AWF::FileReader sim_frame_file(dir_name + "sim_frame_kpts.csv"),
			sim_rframe_file(dir_name + "sim_rframe_kpts.csv"),
			sim_pos_file(dir_name + "sim_pos.csv"),
			sim_qua_file(dir_name + "sim_qua.csv");

	Eigen::MatrixXd sim_frame = sim_frame_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd sim_rframe = sim_rframe_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd sim_pos = sim_pos_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd sim_qua = sim_qua_file.extractDoulbeMatrix(",");

	std::cout << "frame size:" << sim_frame.rows() << " ," << sim_frame.cols() << std::endl;
	std::cout << "rframe size:" << sim_rframe.rows() << "," << sim_rframe.cols() << std::endl;
	std::cout << "pos size:" << sim_pos.rows() << "," << sim_pos.cols() << std::endl;
	std::cout << "qua size:" << sim_qua.rows() << "," << sim_qua.cols() << std::endl;


}