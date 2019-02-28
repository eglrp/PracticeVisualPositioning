//
// Created by steve on 12/30/18.

 #ifndef BASESLAM_UTILTOOLS_H
 #define BASESLAM_UTILTOOLS_H
#include <opencv2/opencv.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>


/**
 * Useful function....
 *
 */

Eigen::Isometry3d rt2Matrix(cv::Vec3d rvec, cv::Vec3d tvec) {
	cv::Mat cv_rotation_matrix;

	cv::Rodrigues(rvec, cv_rotation_matrix);
//    Eigen::Matrix3d rotation_matrix;
	Eigen::Isometry3d transform_matrix(Eigen::Isometry3d::Identity());

	for (int i(0); i < 3; ++i) {
		for (int j(0); j < 3; ++j) {
			transform_matrix(i, j) = cv_rotation_matrix.at<double>(i, j);
		}
	}

	for (int i(0); i < 3; ++i) {
		transform_matrix(i, 3) = tvec(i);
	}
	return transform_matrix;
}


/**
 * @brief out put edge se3.
 * @param os
 * @param to_index
 * @param from_index
 * @param iso_mat
 * @param pos_sigma
 * @param ang_sigma
 * @return
 */
bool outEdgeSE3(std::fstream &os,
                int to_index,
                int from_index,
                Eigen::Isometry3d iso_mat,
                double pos_sigma,
                double ang_sigma) {
	os << "EDGE_SE3:QUAT " << from_index
	   << " " << to_index;
	for (int i = 0; i < 3; ++i) {
		os << " " << iso_mat(i, 3);
	}

	Eigen::Matrix3d rot_mat;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			rot_mat(i, j) = iso_mat(i, j);
		}
	}
	Eigen::Quaterniond q(rot_mat);
	os << " " << q.x()
	   << " " << q.y()
	   << " " << q.z()
	   << " " << q.w();

	Eigen::Matrix<double,6,6> info_mat ;
	for(int i=0;i<6;++i){
		for(int j=0;j<6;++j){
			info_mat(i,j) = 0.0;
			if(i==j && i < 3){
				info_mat(i,j) = 1.0 / pos_sigma;
			}
			if(i==j && i>=3){
				info_mat(i,j) = 1.0 / ang_sigma;
			}
			os << " " << info_mat(i,j);
		}
	}
	os << std::endl;


}

#endif //BASESLAM_UTILTOOLS_H
