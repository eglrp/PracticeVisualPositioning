//
// Created by steve on 4/13/19.
//

#ifndef PRACTICEVISUALPOSITIONING_MARGINALIZATIONFACTOR_H
#define PRACTICEVISUALPOSITIONING_MARGINALIZATIONFACTOR_H

#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>


#include <StereoVO/MarginalizationServer.h>

class MarginalizationFactor : public ceres::CostFunction {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	MarginalizationFactor(std::vector<ParameterBlockInfo *> block_info_vec,
	                      Eigen::MatrixXd &block_linearized_jac,
	                      Eigen::MatrixXd &block_linear_residual) :
			block_linearized_jac_(block_linearized_jac),
			block_linearized_residual_(block_linear_residual) {
		block_info_vec_ = block_info_vec;
	}

	virtual bool Evaluate(double const *const *parameters,
	                      double *residuals, double **jacobians) const {

		int n = block_linearized_jac_.cols();
		int m = block_linearized_jac_.rows();
		Eigen::VectorXd dx(n);
		for(int i=0;i<block_info_vec_.size();++i){
			int size = block_info_vec_[i]->block_size;
			int idx = block_info_vec_[i]->block_idx;
			Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i],size);
			Eigen::VectorXd &x0 = block_info_vec_[i]->keeped_block_value;

		}

	}

	std::vector<ParameterBlockInfo *> block_info_vec_;

	Eigen::MatrixXd block_linearized_jac_;
	Eigen::MatrixXd block_linearized_residual_;
};


#endif //PRACTICEVISUALPOSITIONING_MARGINALIZATIONFACTOR_H
