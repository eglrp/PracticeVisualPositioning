//
// Created by steve on 4/3/19.
//

#ifndef PRACTICEVISUALPOSITIONING_SIMPLEINVDEPTHREPROJECTIONERROR_H
#define PRACTICEVISUALPOSITIONING_SIMPLEINVDEPTHREPROJECTIONERROR_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>

struct SimpleInvDepthReProjectionError {

	SimpleInvDepthReProjectionError(
			double fx,
			double fy,
			double cx,
			double cy,
			double u_i,
			double v_i,
			double u_j,
			double v_j,
			double *q_bc_i,
			double *t_bc_i,
			double *q_bc_j,
			double *t_bc_j
	) {

		q_bc_i_ = Eigen::Map<const Eigen::Quaterniond>(q_bc_i);
		q_bc_j_ = Eigen::Map<const Eigen::Quaterniond>(q_bc_j);
		t_bc_i_ = Eigen::Map<const Eigen::Vector3d>(t_bc_i);
		t_bc_j_ = Eigen::Map<const Eigen::Vector3d>(t_bc_j);


		ob_i_(0) = (u_i - cx) / fx;
		ob_i_(1) = (v_i - cy) / fy;
		ob_i_(2) = 1.0;

		ob_j_(0) = (u_j - cx) / fx;
		ob_j_(1) = (v_j - cy) / fy;
		ob_j_(2) = 1.0;

	}

	template<typename T>
	bool operator()(
			const T *const qi,//4
			const T *const ti,//3
			const T *const qj,//4
			const T *const tj,//3
			const T *const inv_d_ptr,//1
			T *residuals// 2
	) const {
		printf("into the residual problem\n");
		Eigen::Matrix<T, 3, 1> pt_ci_unit, pt_cj_unit;
		pt_ci_unit << T(ob_i_(0)), T(ob_i_(1)), T(ob_i_(2));
		pt_cj_unit << T(ob_j_(0)), T(ob_j_(1)), T(ob_j_(2));

		Eigen::Quaternion<T> q_bc_i(T(q_bc_i_.w()), T(q_bc_i.x()), T(q_bc_i.y()), T(q_bc_i.z()));
		Eigen::Quaternion<T> q_bc_j(T(q_bc_j_.w()), T(q_bc_j.x()), T(q_bc_j.y()), T(q_bc_j.z()));
		Eigen::Matrix<T, 3, 1> t_bc_i(T(t_bc_i_(0)), T(t_bc_i_(1)), T(t_bc_i_(2)));
		Eigen::Matrix<T, 3, 1> t_bc_j(T(t_bc_j_(0)), T(t_bc_j_(1)), T(t_bc_j_(2)));

		Eigen::Map<const Eigen::Quaternion<T>> q_bw_i(qi);
		Eigen::Map<const Eigen::Quaternion<T>> q_bw_j(qj);
		Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_bw_i(ti);
		Eigen::Map<const Eigen::Matrix<T, 3, 1>> t_bw_j(tj);


		Eigen::Matrix<T, 3, 1> pt_w = q_bw_i * (q_bc_i.inverse() * (pt_ci_unit / inv_d_ptr[0] - t_bc_i)) +
		                              t_bw_i; // pt_ci ==> pt_body_i == > pt_w
		Eigen::Matrix<T, 3, 1> pt_cj = q_bc_j * (q_bw_j.inverse() * (pt_w - t_bw_j)) + t_bc_j;//
		Eigen::Matrix<T, 2, 1> pre_pt_cj_unit = pt_cj.block(0, 0, 2, 1) / pt_cj(2);

		Eigen::Map<Eigen::Matrix<T, 2, 1>> residual_vector(residuals);
		Eigen::Matrix<T, 2, 2> sqrt_info_mat;
		sqrt_info_mat << T(sqrt_info(0, 0)), T(sqrt_info(0, 1)), T(sqrt_info(1, 0)), T(sqrt_info(1, 1));

		residual_vector = sqrt_info_mat * (pre_pt_cj_unit - pt_cj_unit.block(0, 0, 2, 1));

		printf("into the residual problem end\n");
		return true;
	}

	static ceres::CostFunction *Create(
			double fx,
			double fy,
			double cx,
			double cy,
			double u_i,
			double v_i,
			double u_j,
			double v_j,
			double *q_bc_i,
			double *t_bc_i,
			double *q_bc_j,
			double *t_bc_j
	) {
		return (new ceres::AutoDiffCostFunction<SimpleInvDepthReProjectionError, 2, 4, 3, 4, 3, 1>(
				new SimpleInvDepthReProjectionError(fx, fy, cx, cy, u_i, v_i, u_j, v_j, q_bc_i, t_bc_i, q_bc_j, t_bc_j)
		));
	}

	Eigen::Vector3d ob_i_, ob_j_;

	Eigen::Quaterniond q_bc_i_; // i-th quaternion from body to camera.
	Eigen::Quaterniond q_bc_j_;
	Eigen::Vector3d t_bc_i_;
	Eigen::Vector3d t_bc_j_;

	Eigen::Matrix2d sqrt_info = Eigen::Matrix2d::Identity() / 5.0; // infomation matrix of observation.

};


#endif //PRACTICEVISUALPOSITIONING_SIMPLEINVDEPTHREPROJECTIONERROR_H
