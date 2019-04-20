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

	MarginalizationFactor(
			std::vector<int> &block_sizes_vec,
			std::vector<int> &block_ids_vec,
			std::vector<Eigen::VectorXd> &keeped_data_vec,
			Eigen::MatrixXd &block_linearized_jac,
			Eigen::MatrixXd &block_linear_residual) :
			block_linearized_jac_(block_linearized_jac),
			block_linearized_residual_(block_linear_residual) {
		for (int &block_size:block_sizes_vec) {
			mutable_parameter_block_sizes()->push_back(block_size);
			block_size_vec_.push_back(block_size);
		}

		for (int &block_idx:block_ids_vec) {
			block_idx_vec_.push_back(block_idx);
		}
		for (auto &data_vector:keeped_data_vec) {
			keeped_data_vec_.push_back(data_vector * 1.0);
		}
		assert(block_size_vec_.size() == block_idx_vec_.size());
		assert(keeped_data_vec_.size() == block_size_vec_.size());

		set_num_residuals(block_linear_residual.rows());// set residual module size.
	}


	virtual bool Evaluate(double const *const *parameters,
	                      double *residuals, double **jacobians) const {

		int n = block_linearized_jac_.cols();
		int m = block_linearized_jac_.rows();
		Eigen::VectorXd dx(n);


		for (int i = 0; i < block_idx_vec_.size(); ++i) {
//			std::cout << "address [" << i << "]: " << parameters[i] << "\n";
			int idx = block_idx_vec_[i];
			int size = block_size_vec_[i];
//			assert(mutable_parameter_block_sizes()->[i]==size);

			if (size == 4) {
				// quaternion
				Eigen::Map<const Eigen::Quaterniond> q(parameters[i]);
				const Eigen::Map<Eigen::Quaterniond> q0(const_cast<double *>(keeped_data_vec_[i].data()));

				Eigen::Quaterniond q_diff = q0.inverse() * q;
				if (q_diff.w() > 0) {
					dx.segment(idx, size) = q_diff.coeffs();
				} else {
					dx.segment(idx, size) = q_diff.coeffs() * -1.0;
				}
				dx.segment(idx,size) = q_diff.coeffs() * 0.0;


			} else {
				// normal
				Eigen::Map<const Eigen::VectorXd> x(parameters[i], size);
//				Eigen::Map<Eigen::VectorXd> x0(para_info.keeped_block_value.data(), size);
				Eigen::VectorXd x0 = keeped_data_vec_[i];
				std::cout << "x - x0:" << (x - x0).transpose()
				          << "x - x0 rows:" << (x - x0).rows()
				          << " dx rows:" << dx.rows()
				          << "idx:" << idx << "size:" << size << "\n";
				dx.segment(idx, size) = x - x0;

			}
		}
		Eigen::Map<Eigen::VectorXd> residual_vector(residuals, n);
		residual_vector = block_linearized_residual_ +
		                  block_linearized_jac_ * dx;

		if (jacobians) {
			int valid_jac_counter=0;
			for (int i = 0; i < block_size_vec_.size(); ++i) {
				if (jacobians[i]) {
					int idx = block_idx_vec_[i];
					int size = block_size_vec_[i];
					Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
							Eigen::ColMajor>> jaco_mat(jacobians[i], n, size);
					jaco_mat.setZero();
					jaco_mat.leftCols(size) = block_linearized_jac_.middleCols(
							idx, size
					);
					if(std::isfinite(jaco_mat.sum())){
						valid_jac_counter+=1;
					}else{
						jaco_mat.setZero();
					}
					if(size == 4){
						jaco_mat.setZero();
					}

				}
			}
			if(valid_jac_counter<10){
				std::cout << "valid jacobian matrix is less than 10, is :" << valid_jac_counter << std::endl;
				return false;
			}


		}
		return true;

	}


	std::vector<int> block_size_vec_;
	std::vector<int> block_idx_vec_;
	std::vector<Eigen::VectorXd> keeped_data_vec_;

	Eigen::MatrixXd block_linearized_jac_;
	Eigen::MatrixXd block_linearized_residual_;
};


#endif //PRACTICEVISUALPOSITIONING_MARGINALIZATIONFACTOR_H
