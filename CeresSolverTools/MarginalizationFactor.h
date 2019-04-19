//
// Created by steve on 4/13/19.
//

#ifndef PRACTICEVISUALPOSITIONING_MARGINALIZATIONFACTOR_H
#define PRACTICEVISUALPOSITIONING_MARGINALIZATIONFACTOR_H

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>

//#include <StereoVO/MarginalizationServer.h>
#include <StereoVO/MarginalizationInfo.h>

class MarginalizationFactor : public ceres::CostFunction {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	MarginalizationFactor(std::map<double *, ParameterBlockInfo> *map_ptr,
	                      std::vector<int> &block_sizes_vec,
	                      Eigen::MatrixXd &block_linearized_jac,
	                      Eigen::MatrixXd &block_linear_residual) :
			block_linearized_jac_(block_linearized_jac),
			block_linearized_residual_(block_linear_residual) {
		ptr_info_map_ptr_ = map_ptr;
		for (int &block_size:block_sizes_vec) {
			mutable_parameter_block_sizes()->push_back(block_size);
		}

		set_num_residuals(block_linear_residual.rows());// set residual module size.
	}

	virtual bool Evaluate(double const *const *parameters,
	                      double *residuals, double **jacobians) const {

		int n = block_linearized_jac_.cols();
		int m = block_linearized_jac_.rows();
		Eigen::VectorXd dx(n);
		for (int i = 0; i < ptr_info_map_ptr_->size(); ++i) {
			auto &para_info = ptr_info_map_ptr_->
					find(const_cast<double *>(parameters[i]))->second;
			int idx = para_info.block_idx;
			int size = para_info.global_block_size;
//			assert(mutable_parameter_block_sizes()->[i]==size);

			if (para_info.global_block_size == 4) {
				// quaternion
				Eigen::Map<const Eigen::Quaterniond> q(parameters[i]);
				Eigen::Map<Eigen::Quaterniond> q0(para_info.keeped_block_value.data());

				Eigen::Quaterniond q_diff = q0.inverse() * q;
				if (q_diff.w() > 0) {
					dx.segment(idx, size) = q_diff.coeffs();
				} else {
					dx.segment(idx, size) = q_diff.coeffs() * -1.0;
				}


			} else {
				// normal
				Eigen::Map<const Eigen::VectorXd> x(parameters[i], size);
				Eigen::Map<Eigen::VectorXd> x0(para_info.keeped_block_value.data(), size);
//				std::cout << "flag:"<< para_info.removed_flag << "\n";
//				std::cout << "x - x0:" << (x - x0).transpose()
//				          << "x - x0 rows:" << (x - x0).rows()
//				          << " dx rows:" << dx.rows()
//				          << "idx:" << idx << "size:" << size << "\n";
				dx.segment(idx, size) = x - x0;

			}
		}
		Eigen::Map<Eigen::VectorXd> residual_vector(residuals, n);
		residual_vector = block_linearized_residual_ +
		                  block_linearized_jac_ * dx;

		if (jacobians) {
			for (int i = 0; i < ptr_info_map_ptr_->size(); ++i) {
				if (jacobians[i]) {
					ParameterBlockInfo &para_info = ptr_info_map_ptr_->
							find(const_cast<double *>(parameters[i]))->second;
					int idx = para_info.block_idx;
					int size = para_info.global_block_size;
					Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
							Eigen::ColMajor>> jaco_mat(jacobians[i], n, size);
					jaco_mat.setZero();
					jaco_mat.leftCols(size) = block_linearized_jac_.middleCols(
							idx, size
					);

				}
			}

		}
		return true;

	}

	std::map<double *, ParameterBlockInfo> *ptr_info_map_ptr_;

	Eigen::MatrixXd block_linearized_jac_;
	Eigen::MatrixXd block_linearized_residual_;
};


#endif //PRACTICEVISUALPOSITIONING_MARGINALIZATIONFACTOR_H
