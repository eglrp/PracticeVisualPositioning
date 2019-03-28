//
// Created by steve on 3/26/19.
//

#ifndef PRACTICEVISUALPOSITIONING_GEOTOOLS_H
#define PRACTICEVISUALPOSITIONING_GEOTOOLS_H

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>


/**
 * @brief
 * @param Pose0
 * @param Pose1
 * @param point0
 * @param point1
 * @param point_3d
 * @return
 */
inline bool triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                             Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d) {
	Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
	design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
	design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
	design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
	design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
	Eigen::Vector4d triangulated_point;
	triangulated_point =
			design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
	point_3d(0) = triangulated_point(0) / triangulated_point(3);
	point_3d(1) = triangulated_point(1) / triangulated_point(3);
	point_3d(2) = triangulated_point(2) / triangulated_point(3);

//	cv::Mat C0(3,4,CV_64F),C1(3,4,CV_64F);
//
//	for(int i=0;i<3;++i){
//		for(int j=0;j<4;++j){
//			C0.at<double>(i,j) = ()
//		}
//	}
//
//
//	cv::triangulatePoints()

//	std::cout << "------------------------\n"
//	          << Pose0(0, 3) << "," << Pose0(1, 3) << "," << Pose0(2, 3) << "\n"
//	          << Pose1(0, 3) << "," << Pose1(1, 3) << "," << Pose1(2, 3) << "\n"
//	          << point0(0) << "," << point0(1) << "\n"
//	          << point1(0) << "," << point1(1) << "\n"
//	          << point_3d(0) << "," << point_3d(1) << "," << point_3d(2)
//	          << "\n-----------------------\n" << std::endl;

	return true;
}

/**
 * @brief calculate 3d position of a point observed by two frame(0 and 1).
 * @param R0 rotation matrix of 0 moment
 * @param t0  translation vector of 0 moment.
 * @param R1
 * @param t1
 * @param pt0  points (2d)
 * @param pt1
 * @param pt3d 3D position of points.
 * @return
 */
inline bool triangulatePointRt(Eigen::Matrix<double, 3, 3> &R0, Eigen::Matrix<double, 3, 1> &t0,
                               Eigen::Matrix<double, 3, 3> &R1, Eigen::Matrix<double, 3, 1> &t1,
                               cv::Mat cam_mat, cv::Mat dist_coeff,
                               Eigen::Vector2d &pt0, Eigen::Vector2d &pt1,
                               Eigen::Vector3d &pt3d) {
	Eigen::Matrix<double, 3, 4> pose0, pose1;
	pose0.block(0, 0, 3, 3) = R0 * 1.0;
	pose0.block(0, 3, 3, 1) = t0 * 1.0;
	pose1.block(0, 0, 3, 3) = R1 * 1.0;
	pose1.block(0, 3, 3, 1) = t1 * 1.0;


	cv::Mat C0(3, 4, CV_32F), C1(3, 4, CV_32F);
	cv::Mat cvpt0(2, 1, CV_32F), cvpt1(2, 1, CV_32F);
	cv::Mat p4d;


	for (int i(0); i < 2; ++i) {
		cvpt0.at<float>(i, 0) = float(pt0(i));
		cvpt1.at<float>(i, 0) = float(pt1(i));
	}

	for (int i(0); i < 3; ++i) {
		for (int j(0); j < 4; ++j) {

			C0.at<float>(i, j) = 0.0;
			C1.at<float>(i, j) = 0.0;
			for (int k(0); k < 3; ++k) {
				C0.at<float>(i, j) += (cam_mat.at<float>(i, k) * float(pose0(k, j)));
				C1.at<float>(i, j) += (cam_mat.at<float>(i, k) * float(pose1(k, j)));

			}
		}
	}
	cv::triangulatePoints(
			C0, C1, cvpt0, cvpt1, p4d
	);

	for (int i = 0; i < 3; ++i) {
		pt3d(i) = double(p4d.at<float>(i, 0));
	}

	std::cout << "------------------------\n"
	          << t0(0) << "," << t0(1) << "," << t0(2) << "\n"
	          << t1(0) << "," << t1(1) << "," << t1(2) << "\n"
	          << pt0(0) << "," << pt0(1) << "\n"
	          << pt1(0) << "," << pt1(1) << "\n"
	          << pt3d(0) << "," << pt3d(1) << "," << pt3d(2)
	          << "\n-----------------------\n" << std::endl;

	return true;


}


/**
 * @brief Solve pose by observed 3d point in 2d image.
 * @param qua_ini  initial quaternion
 * @param t_ini initial translation
 * @param ob_pt observed points (2d) in image
 * @param pts3 3D position of points
 * @param cam_mat camera matrix
 * @param dist_coeff distort coefficient matrix.
 * @return
 */
inline bool solvePosePnp(Eigen::Quaterniond &qua_ini,
                         Eigen::Vector3d &t_ini,
                         std::vector<cv::Point2f> &ob_pt,
                         std::vector<cv::Point3f> &pts3,
                         cv::Mat &cam_mat,
                         cv::Mat &dist_coeff) {
	assert(ob_pt.size() == pts3.size());

	if (ob_pt.size() < 5) {
		return false;
	}

	bool pnp_succ;

	cv::Mat rvec, tvec, tmp_r;

	cv::eigen2cv(qua_ini.toRotationMatrix(), tmp_r);
	cv::Rodrigues(tmp_r, rvec);
	cv::eigen2cv(t_ini, tvec);


	pnp_succ = cv::solvePnPRansac(pts3, ob_pt, cam_mat, dist_coeff,
	                              rvec, tvec,
	                              false,
	                              100,
	                              8.0,
	                              0.99);
//	pnp_succ = cv::solvePnP(
//			pts3, ob_pt, cam_mat, dist_coeff, rvec, tvec
//	);
	if (!pnp_succ) {

		return false;
	} else {
		Eigen::MatrixXd r;
		Eigen::VectorXd t;
		cv::Mat trvec;
		cv::Rodrigues(rvec, trvec);
		cv::cv2eigen(trvec, r);
		cv::cv2eigen(tvec, t);
		qua_ini = Eigen::Matrix3d(r);
		t_ini = t;
		return true;
	}
}

#endif //PRACTICEVISUALPOSITIONING_GEOTOOLS_H
