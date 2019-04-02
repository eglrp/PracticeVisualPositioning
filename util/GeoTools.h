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

#include <ceres/ceres.h>
#include <ceres/rotation.h>

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
//	cv::Mat cvpt0(2, 1, CV_32F), cvpt1(2, 1, CV_32F);
	cv::Mat p4d(4, 1, CV_32F);


	std::vector<cv::Point2f> pts0, pts1;
	std::vector<cv::Point2f> unpts0, unpts1;
	pts0.push_back(cv::Point2f(pt0.x(), pt0.y()));
	pts1.push_back(cv::Point2f(pt1.x(), pt1.y()));

//	cv::Mat icam(3,3, CV_32F,cv::Scalar(0.0));
//	icam.at<float>(0,0) = 1.0;
//	icam.at<float>(1,1) = 1.0;
//	icam.at<float>(0,2) = 0.0;
//	icam.at<float>(1,2) = 0.0;
//	icam.at<float>(2,2) = 1.0;

	cv::undistortPoints(pts0, unpts0, cam_mat, dist_coeff);//,icam);
	cv::undistortPoints(pts1, unpts1, cam_mat, dist_coeff);//,icam);


	for (int i(0); i < 3; ++i) {
		for (int j(0); j < 3; ++j) {
//			C0.at<float>(i, j) = 0.0;
//			C1.at<float>(i, j) = 0.0;
//			for (int k(0); k < 3; ++k) {
//				C0.at<float>(i, j) += (cam_mat.at<float>(i, k) * float(pose0(k, j)));
//				C1.at<float>(i, j) += (cam_mat.at<float>(i, k) * float(pose1(k, j)));
//			}
			C0.at<float>(i, j) = float(R0(i, j));
			C1.at<float>(i, j) = float(R1(i, j));
		}
		C0.at<float>(i, 3) = float(t0(i, 0));
		C1.at<float>(i, 3) = float(t1(i, 0));
	}
	cv::triangulatePoints(
			C0, C1, unpts0, unpts1, p4d
	);

	for (int i = 0; i < 3; ++i) {
		pt3d(i) = double(p4d.at<float>(i, 0) / double(p4d.at<float>(3, 0)));// / p4d.at<float>(3,0));
	}

	if (pt3d(2) < 0.0) {
		pt3d *= -1.0;
	}

	if (pt3d(2) > 3000.0) {
		return false;

	}

	return true;


}

struct ProjectionPnPFactor {
	ProjectionPnPFactor(
			double fx,
			double fy,
			double cx,
			double cy,
			double obx,
			double oby,
			double *pt3d
	) : fx_(fx), fy_(fy), cx_(cx), cy_(cy), obx_(obx), oby_(oby) {
		pt3d_ = pt3d;
	}


	template<typename T>
	bool operator()(
			const T *const qwc,
			const T *const twc,
			T *residuals) const {
		T p[3];
		T pt3[3];
		for (int i = 0; i < 3; ++i) {
			pt3[i] = T(pt3d_[i]);
		}
		ceres::QuaternionRotatePoint(qwc, pt3, p);
		p[0] += twc[0];
		p[1] += twc[1];
		p[2] += twc[2];


		T xp = p[0] / p[2];
		T yp = p[1] / p[2];

		T fx = T(fx_);
		T fy = T(fy_);
		T cx = T(cx_);
		T cy = T(cy_);

		T pre_x = fx * xp + cx;
		T pre_y = fy * yp + cy;

		residuals[0] = pre_x - T(obx_);
		residuals[1] = pre_y - T(oby_);
		return true;
	}

	static ceres::CostFunction *Create(
			double fx,
			double fy,
			double cx,
			double cy,
			double obx,
			double oby,
			double *pt3d
	) {
		return (new ceres::AutoDiffCostFunction<ProjectionPnPFactor, 2, 4, 3>(
				new ProjectionPnPFactor(fx, fy, cx, cy, obx, oby, pt3d)
		));
	}


	double *pt3d_;
	double fx_, fy_, cx_, cy_, obx_, oby_;

};


inline bool solvePosePnpCeres(
		Eigen::Quaterniond &qua_ini,
		Eigen::Vector3d &t_ini,
		Eigen::Quaterniond &q_bc,
		Eigen::Vector3d &t_bc,
		std::vector<cv::Point2f> &ob_pt,
		std::vector<cv::Point3f> &pts3d,
		cv::Mat &cam_mat,
		cv::Mat &dist_coeff
) {

	Eigen::Quaterniond qua = q_bc * qua_ini.inverse();
	Eigen::Vector3d t0 = (qua * (-1.0 * t_ini)) + t_bc;

	ceres::Problem problem;
	ceres::Solver::Options options;
	ceres::Solver::Summary summary;

	options.linear_solver_type = ceres::DENSE_QR;

	double fx = double(cam_mat.at<float>(0, 0));
	double fy = double(cam_mat.at<float>(1, 1));
	double cx = double(cam_mat.at<float>(0, 2));
	double cy = double(cam_mat.at<float>(1, 2));

	double ob3d_array[3 * ob_pt.size()];

	double qua_array[4] = {qua.w(), qua.x(), qua.y(), qua.z()};
	problem.AddParameterBlock(
			qua_array,
			4,
			new ceres::QuaternionParameterization()
	);

	for (int i = 0; i < ob_pt.size(); ++i) {
		ob3d_array[i * 3 + 0] = double(pts3d[i].x);
		ob3d_array[i * 3 + 1] = double(pts3d[i].y);
		ob3d_array[i * 3 + 2] = double(pts3d[i].z);

		problem.AddResidualBlock(
				ProjectionPnPFactor::Create(
						fx, fy, cx, cy, double(ob_pt[i].x),
						double(ob_pt[i].y),
						ob3d_array + i * 3
				),
				new ceres::CauchyLoss(5.0),
//				NULL,
				qua_array,
				t0.data()
		);

	}
	options.linear_solver_type=ceres::DENSE_SCHUR;
	if(ob_pt.size()>12){
		options.num_linear_solver_threads=6;
		options.num_threads  =6;
	}

	ceres::Solve(options, &problem, &summary);
//	std::cout << summary.FullReport() << std::endl;

	qua = Eigen::Quaterniond(qua_array[0], qua_array[1], qua_array[2], qua_array[3]).normalized();

	qua_ini = qua.inverse() * q_bc;
	t_ini = -1.0 * (qua.inverse().toRotationMatrix() * (t0 - t_bc));

	if(t_ini.norm() > 1e4){
		std::cout << "t_ini:" << t_ini.transpose() << std::endl;
		std::cout << "qua_ini:" << qua_ini.matrix() << std::endl;

		for(int i(0);i<pts3d.size();++i){
			std::cout << pts3d[i] << " ===>" << ob_pt[i] << std::endl;
		}


		std::cout << "SOME ERROR HAPPEND " << std::endl;
	}

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
		std::cout << "SOME ERROR HAPPEND>>>" << std::endl;


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

		std::cout << "pns t:" << t_ini << std::endl;
		return true;
	}
}

struct ProjectionKnowCamFactor {
	ProjectionKnowCamFactor(double *q0,
	                        double *t0,
	                        double fx,
	                        double fy,
	                        double cx,
	                        double cy,
	                        double obx,
	                        double oby) :
			fx_(fx), fy_(fy), cx_(cx), cy_(cy), obx_(obx), oby_(oby) {
		q0_ = q0;
		t0_ = t0;
	}

	template<typename T>
	bool operator()(
			const T *const pt,
			T *residuals
	) const {
		T pr[3];
		T p[3];
		T q[4];
		for (int i = 0; i < 4; ++i) {
			q[i] = T(q0_[i]);
		}


		ceres::QuaternionRotatePoint(q, pt, p);
		p[0] += T(t0_[0]);
		p[1] += T(t0_[1]);
		p[2] += T(t0_[2]);

		T xp = p[0] / p[2];
		T yp = p[1] / p[2];

		T fx = T(fx_);
		T fy = T(fy_);
		T cx = T(cx_);
		T cy = T(cy_);

		T pre_x = fx * xp + cx;
		T pre_y = fy * yp + cy;

		if (ceres::IsNaN(pre_x) || ceres::IsNaN(pre_y)) {
			std::cout << " p :" << p[0] << "," << p[1] << "," << p[2] << std::endl;
			std::cout << " xp:" << xp << "\n yp:" << yp << std::endl;
			std::cout << "pre x:" << pre_x << " \npre y:" << pre_y << std::endl;

			std::cout << "residula:" << pre_x - T(obx_) << ";"
			          << pre_y - T(oby_) << std::endl;

		}

		residuals[0] = pre_x - T(obx_);
		residuals[1] = pre_y - T(oby_);
		return true;


	}

	static ceres::CostFunction *Create(double *q0,
	                                   double *t0,
	                                   double fx,
	                                   double fy,
	                                   double cx,
	                                   double cy,
	                                   double obx,
	                                   double oby) {
		return (new ceres::AutoDiffCostFunction<ProjectionKnowCamFactor, 2, 3>(
				new ProjectionKnowCamFactor(q0, t0, fx, fy, cx, cy, obx, oby)
		));
	}


	double *q0_;
	double *t0_;
	double fx_, fy_, cx_, cy_, obx_, oby_;

};

inline bool triangulatePointEigen(Eigen::Quaterniond q0, Eigen::Matrix<double, 3, 1> t0,
                                  Eigen::Quaterniond q1, Eigen::Matrix<double, 3, 1> t1,
                                  cv::Mat cam_mat,
                                  Eigen::Vector2d &pt0, Eigen::Vector2d &pt1, Eigen::Vector3d &pt3d) {

	Eigen::Matrix<double, 6, 4> designed_matrix;
	Eigen::Matrix<double, 6, 1> designed_vec;

	designed_vec(0) = pt0.x();
	designed_vec(1) = pt0.y();
	designed_vec(2) = 1.0;
	designed_vec(3) = pt1.x();
	designed_vec(4) = pt1.y();
	designed_vec(5) = 1.0;


	Eigen::Matrix<double, 3, 3> K;
	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			K(i, j) = double(cam_mat.at<float>(i, j));
		}
	}

	designed_matrix.block(0, 0, 3, 3) = q0.toRotationMatrix();
	designed_matrix.block(0, 3, 3, 1) = t0;

	designed_matrix.block(3, 0, 3, 3) = q1.toRotationMatrix();
	designed_matrix.block(3, 3, 3, 1) = t1;


	designed_matrix.block(0, 0, 3, 4) = K * designed_matrix.block(0, 0, 3, 4);
	designed_matrix.block(3, 0, 3, 4) = K * designed_matrix.block(3, 0, 3, 4);

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(designed_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);//M=USV*
	double pinvtoler = 1.e-8; //tolerance
	int row = designed_matrix.rows();
	int col = designed_matrix.cols();
	int k = std::min(row, col);
	Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col, row);
	Eigen::MatrixXd singularValues_inv = svd.singularValues();//奇异值
	Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
	for (long i = 0; i < k; ++i) {
		if (singularValues_inv(i) > pinvtoler)
			singularValues_inv(i) = 1.0 / singularValues_inv(i);
		else singularValues_inv(i) = 0;
	}
	for (long i = 0; i < k; ++i) {
		singularValues_inv_mat(i, i) = singularValues_inv(i);
	}
	X = (svd.matrixV()) * (singularValues_inv_mat) * (svd.matrixU().transpose());//X=VS+U*
	Eigen::Vector4d s_pts = X * designed_vec;

	pt3d(0) = s_pts(0) / s_pts(3);
	pt3d(1) = s_pts(1) / s_pts(3);
	pt3d(2) = s_pts(2) / s_pts(3);

	if ((q0 * pt3d + t0)(2) < 0.0) {
		pt3d = q0.inverse() * (q0 * pt3d + t0) * -1.0 - t0;
	}

	return true;


}

/**
 * @brief Passed test
 * @param q0  quaternion from word to image frame.
 * @param t0  t from world to image.
 * @param q1
 * @param t1
 * @param cam_mat
 * @param pt0
 * @param pt1
 * @param pt3d
 * @return
 */
inline bool triangulatePointCeres(Eigen::Quaterniond q0, Eigen::Matrix<double, 3, 1> t0,
                                  Eigen::Quaterniond q1, Eigen::Matrix<double, 3, 1> t1,
                                  cv::Mat cam_mat,
                                  Eigen::Vector2d &pt0, Eigen::Vector2d &pt1,
                                  Eigen::Vector3d &pt3d) {
	ceres::Problem problem;
	ceres::Solver::Options option;
	ceres::Solver::Summary summary;

	option.linear_solver_type = ceres::DENSE_QR;

	double qua0[4] = {q0.w(), q0.x(), q0.y(), q0.z()};
	double qua1[4] = {q1.w(), q1.x(), q1.y(), q1.z()};

	double fx = double(cam_mat.at<float>(0, 0));
	double fy = double(cam_mat.at<float>(1, 1));
	double cx = double(cam_mat.at<float>(0, 2));
	double cy = double(cam_mat.at<float>(1, 2));

	double xp, yp;
	xp = (pt0.x() - cx) / fx;
	yp = (pt0.y() - cy) / fy;

	double Z = 10.0;
	if (abs(t0.x() - t1.x()) > 0.2) {

		Z = (t0 - t1).norm() * sqrt(fx * fx + fy * fy) / (pt0 - pt1).norm();
	} else {
		Z = 10.0;

	}
//
	double X = xp * Z;
	double Y = yp * Z;
//
	pt3d = q0.inverse().toRotationMatrix() * (Eigen::Vector3d(X, Y, Z) - t0);
//
//	pt3d = q0.toRotationMatrix() * (Eigen::Vector3d(X,Y,Z) + t0);
	problem.AddResidualBlock(
			ProjectionKnowCamFactor::Create(
					qua0,
					t0.data(),
					fx,
					fy,
					cx,
					cy,
					pt0.x(),
					pt0.y()
			),
			NULL,
			pt3d.data()
	);

	problem.AddResidualBlock(
			ProjectionKnowCamFactor::Create(
					qua1,
					t1.data(),
					fx,
					fy,
					cx,
					cy,
					pt1.x(),
					pt1.y()
			),
			NULL,
			pt3d.data()
	);


	ceres::Solve(option, &problem, &summary);

//	std::cout << "triangulated:"<< pt3d.transpose() << std::endl;
//	Eigen::Vector3d rt_pt = q0.toRotationMatrix() *



//	std::cout << summary.FullReport() << std::endl;


	return true;
}


#endif //PRACTICEVISUALPOSITIONING_GEOTOOLS_H
