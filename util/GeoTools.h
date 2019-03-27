//
// Created by steve on 3/26/19.
//

#ifndef PRACTICEVISUALPOSITIONING_GEOTOOLS_H
#define PRACTICEVISUALPOSITIONING_GEOTOOLS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

/** define **/

bool triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                      Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d);

//
//bool triangulatePoint(Eigen::Matrix<double, 4, 4> &Pose0,
//                      Eigen::Matrix<double, 4, 4> &Pose1,
//                      Eigen::Vector2d &point0,
//                      Eigen::Vector2d &point1,
//                      Eigen::Vector3d &point_3d);

bool triangulatePoint(Eigen::Matrix<double, 3, 3> &R0, Eigen::Matrix<double, 3, 1> &t0,
                      Eigen::Matrix<double, 3, 3> &R1, Eigen::Matrix<double, 3, 1> &t1,
                      Eigen::Vector2d &pt0, Eigen::Vector2d &pt1,
                      Eigen::Vector3d &pt3d);


bool triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
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
bool triangulatePoint(Eigen::Matrix<double, 3, 3> &R0, Eigen::Matrix<double, 3, 1> &t0,
                      Eigen::Matrix<double, 3, 3> &R1, Eigen::Matrix<double, 3, 1> &t1,
                      Eigen::Vector2d &pt0, Eigen::Vector2d &pt1,
                      Eigen::Vector3d &pt3d) {
	Eigen::Matrix<double, 3, 4> pose0, pose1;
	pose0.block(0, 0, 3, 3) = R0 * 1.0;
	pose0.block(0, 3, 3, 1) = t0 * 1.0;

	pose1.block(0, 0, 3, 3) = R1 * 1.0;
	pose1.block(0, 3, 3, 1) = t1 * 1.0;

	return triangulatePoint(pose0, pose1,
	                        pt0, pt1, pt3d);

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
bool solvePosePnp(Eigen::Quaterniond &qua_ini,
                  Eigen::Vector3d &t_ini,
                  std::vector<cv::Point2f> &ob_pt,
                  std::vector<cv::Point3f> &pts3,
                  cv::Mat &cam_mat,
                  cv::Mat &dist_coeff) {
	assert(ob_pt.size() == pts3.size());

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
	if (!pnp_succ) {

		return false;
	} else {
		Eigen::MatrixXd r;
		Eigen::VectorXd t;
		cv::Mat trvec;
		cv::Rodrigues(rvec, trvec);
		cv::cv2eigen(trvec, r);
		cv::cv2eigen(tvec, t);
		qua_ini = Eigen::Quaterniond(r);
		t_ini = t;
		return true;
	}
}

#endif //PRACTICEVISUALPOSITIONING_GEOTOOLS_H
