//
// Created by steve on 4/13/19.
//

#ifndef PRACTICEVISUALPOSITIONING_INVDEPTHREPROJECTIONERROR_H
#define PRACTICEVISUALPOSITIONING_INVDEPTHREPROJECTIONERROR_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class InvDepthReProjectionError : public ceres::SizedCostFunction<2, 4, 3, 4, 3, 1> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	InvDepthReProjectionError(
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


	virtual bool Evaluate(double const *const *parameters,
	                      double *residuals,
	                      double **jacobians) const {

		Eigen::Map<const Eigen::Quaterniond> q_bw_i(parameters[0]);
		Eigen::Map<const Eigen::Vector3d> t_bw_i(parameters[1]);
		Eigen::Map<const Eigen::Quaterniond> q_bw_j(parameters[2]);
		Eigen::Map<const Eigen::Vector3d> t_bw_j(parameters[3]);
		const double inv_depth = parameters[4][0]; // inverse depth.


		Eigen::Vector3d pt_w = q_bw_i * (q_bc_i_.inverse() * (ob_i_/inv_depth-t_bc_i_)) + t_bw_i;
		Eigen::Vector3d pt_cj = q_bc_j_ * (q_bw_j.inverse() * (pt_w - t_bw_j)) + t_bc_j_;
		Eigen::Vector2d pre_pt_cj_unit = pt_cj.block(0,0,2,1) / pt_cj(2);

		if(pt_cj(2) < 0.1){
			std::cout << "ERROR pt_cj(2) < 0.1" << std::endl;
			return false;
		}

		Eigen::Map<Eigen::Vector2d> residual_vec(residuals);
		residual_vec = sqrt_info * (pre_pt_cj_unit-ob_j_.block(0,0,2,1));

		if(jacobians){
			// calculate jacobian matrix.


		}




		return true;

	}


	Eigen::Vector3d ob_i_, ob_j_;

	Eigen::Quaterniond q_bc_i_; // i-th quaternion from body to camera.
	Eigen::Quaterniond q_bc_j_;
	Eigen::Vector3d t_bc_i_;
	Eigen::Vector3d t_bc_j_;

	Eigen::Matrix2d sqrt_info = Eigen::Matrix2d::Identity() / 5.0; // infomation matrix of observation.
};


#endif //PRACTICEVISUALPOSITIONING_INVDEPTHREPROJECTIONERROR_H
