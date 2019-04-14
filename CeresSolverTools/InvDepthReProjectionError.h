//
// Created by steve on 4/13/19.
//

#ifndef PRACTICEVISUALPOSITIONING_INVDEPTHREPROJECTIONERROR_H
#define PRACTICEVISUALPOSITIONING_INVDEPTHREPROJECTIONERROR_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//// useful function
inline Eigen::Matrix3d hat(Eigen::Vector3d a) {
	Eigen::Matrix3d hat_a;
	hat_a << 0.0, -a.z(), a.y(),
			a.z(), 0.0, -a.x(),
			-a.y(), a.x(), 0.0;
	return hat_a;
}


/**
 * @brief \partical(R{q} a)\partial{q}
 * @param qua
 * @param a
 * @return
 */
inline Eigen::Matrix<double, 3, 4> quternion_derivative(Eigen::Quaterniond qua, Eigen::Vector3d a) {
	double w = qua.w();
	Eigen::Vector3d v(qua.x(), qua.y(), qua.z());

	Eigen::Matrix<double, 3, 4> derivative;
	derivative.block(0, 3, 3, 1) = w * a + v.cross(a);
	derivative.block(0, 0, 3, 3) = v.transpose() * a * Eigen::Matrix3d::Identity() + v * a.transpose()
	                               - a * v.transpose() - w * hat(a);
	return 2.0 * derivative;

}

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


		Eigen::Vector3d pt_bi = q_bc_i_.inverse() * (ob_i_ / inv_depth - t_bc_i_);
		Eigen::Vector3d pt_w = q_bw_i * (pt_bi) + t_bw_i;
		Eigen::Vector3d pt_bj = (q_bw_j.inverse() * (pt_w - t_bw_j));
		Eigen::Vector3d pt_cj = q_bc_j_ * pt_bj + t_bc_j_;
		Eigen::Vector2d pre_pt_cj_unit = pt_cj.block(0, 0, 2, 1) / pt_cj(2);
		double depth_j = pt_cj(2);

		if (pt_cj(2) < 0.1) {
			std::cout << "ERROR pt_cj(2) < 0.1" << std::endl;
			return false;
		}

		Eigen::Map<Eigen::Vector2d> residual_vec(residuals);
		residual_vec = sqrt_info * (pre_pt_cj_unit - ob_j_.block(0, 0, 2, 1));

		if (jacobians) {
			// calculate jacobian matrix.
			Eigen::Matrix3d R_bw_i = q_bw_i.toRotationMatrix();
			Eigen::Matrix3d R_bw_j = q_bw_j.toRotationMatrix();
			Eigen::Matrix3d R_bc_i = q_bc_i_.toRotationMatrix();
			Eigen::Matrix3d R_bc_j = q_bc_j_.toRotationMatrix();

			Eigen::Matrix<double, 2, 3> reduce_mat(Eigen::Matrix<double, 2, 3>::Identity());
			//
			reduce_mat <<
			           1.0 / depth_j, 0, -pt_cj(0) / depth_j / depth_j,
					0.0, 1.0 / depth_j, -pt_cj(1) / depth_j / depth_j;

			reduce_mat = sqrt_info * reduce_mat;

			//q_bw_i
			if (jacobians[0]) {
				Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>
						jacobian_qbw_i(jacobians[0]);
				Eigen::Matrix<double, 3, 4> jacobian_pt_cj_qbw_i = R_bc_j * R_bw_j.transpose() *
				                                                   quternion_derivative(q_bw_i, pt_bi);
				jacobian_qbw_i = reduce_mat * jacobian_pt_cj_qbw_i;
			}


			// t_bw_i
			if (jacobians[1]) {
				Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>>
						jacobian_tbw_i(jacobians[1]);

				jacobian_tbw_i = reduce_mat * R_bc_j * R_bw_j.transpose();
			}

			// q_bw_j
			if (jacobians[2]) {
				Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>
						jacobian_qbw_j(jacobians[2]);
				Eigen::Matrix4d dqinv_dq = Eigen::Matrix4d::Identity();
				dqinv_dq.block(0, 0, 3, 3) *= -1.0;
				Eigen::Matrix<double, 3, 4> jacobian_pt_cj_qbw_j = R_bc_j *
				                                                   quternion_derivative(q_bw_j.inverse(),
				                                                                        pt_w - t_bw_j) *
				                                                   dqinv_dq;
				jacobian_qbw_j = reduce_mat * jacobian_pt_cj_qbw_j;

			}

			// t_bw_j
			if (jacobians[3]) {
				Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>>
						jacobian_tbw_j(jacobians[3]);
				jacobian_tbw_j = reduce_mat * R_bc_j * R_bw_j.transpose() * -1.0;

			}

			// inverse_depth_i
			if (jacobians[4]) {
//				Eigen::Map<Eigen::Matrix<double, 2, 1, Eigen::RowMajor>>
				Eigen::Map<Eigen::Vector2d> jacobian_inv_depth(jacobians[4]);
				jacobian_inv_depth = (reduce_mat * R_bc_j * R_bw_j.transpose() * R_bw_i * R_bc_i.transpose()
				                      * ob_i_ * -1.0 / (inv_depth * inv_depth));

			}


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
