//
// Created by steve on 4/19/19.
//

#ifndef PRACTICEVISUALPOSITIONING_MARGINALIZATIONINFO_H
#define PRACTICEVISUALPOSITIONING_MARGINALIZATIONINFO_H
#include <ceres/ceres.h>

#include <Eigen/Dense>

struct ParameterBlockInfo {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	const double *block_para_address = nullptr;
	// parameter block size at local and global.
	int global_block_size = -1;
	int local_block_size = -1;

	int block_idx = -1;
	int total_idx = -1;
	bool removed_flag = false;

//	double *keeped_block_value = nullptr;
	Eigen::VectorXd keeped_block_value;

//	~ParameterBlockInfo() {
//		delete[] keeped_block_value;
//	}
};

struct ResidualBlockInfo {
	ceres::CostFunction *cost_func_ptr;
	std::vector<double *> para_block_vec;

	std::vector<Eigen::Matrix<double,
			Eigen::Dynamic,
			Eigen::Dynamic,
			Eigen::RowMajor>> jacobian_matrix_vec;

	// TODO:adopting new version of marginalization for speeding up.


};
#endif //PRACTICEVISUALPOSITIONING_MARGINALIZATIONINFO_H
