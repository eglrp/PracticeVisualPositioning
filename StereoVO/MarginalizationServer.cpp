//
// Created by steve on 4/11/19.
//

#include "MarginalizationServer.h"


bool MarginalizationServer::markRemovedParameter(double *para_ptr) {
	if (removed_block_set_.find(para_ptr) != removed_block_set_.end()) {
		// the address of the parameter marked previous.
		return false;
	} else {
		removed_block_set_.insert(para_ptr);
		return true;
	}
}


bool MarginalizationServer::MarignalizationProcess(ceres::Problem &problem) {
	// no parameter block is removed.
	if (removed_block_set_.size() < 1) {
		return false;
	}


	// remove all parameter BLock info imformation
	address_block_info_map_.clear();


	// initial parameter idx.
	int total_dimensional = 0;
	int total_removed_dismensional = 0;

	// recored all parameter block
	std::vector<double *> parameter_block_vec;
	std::vector<int> parameter_size_vec;
	problem.GetParameterBlocks(&parameter_block_vec);

	int remain_index = 0;
	int remove_index = 0;
	int total_index = 0;

	int removed_block_num = 0;
	for (auto &para_ptr:parameter_block_vec) {
		int block_size = problem.ParameterBlockSize(para_ptr);
		int block_local_size = problem.ParameterBlockLocalSize(para_ptr);
		auto removed_flag = (removed_block_set_.find(para_ptr) == removed_block_set_.end()) ? false : true;

		ParameterBlockInfo para_info;
		para_info.global_block_size = block_size;
		para_info.local_block_size = block_local_size;
		para_info.block_para_address = para_ptr;
		para_info.removed_flag = removed_flag;


		if (removed_flag) {
			para_info.block_idx = remove_index;
			removed_block_num++;
			remove_index += block_size;
		} else {
			para_info.block_idx = remain_index;
			remain_index += block_size;
		}
		total_index = remain_index + remove_index;
		para_info.keeped_block_value = Eigen::VectorXd(block_size);
		memcpy(para_info.keeped_block_value.data(), para_ptr, block_size * sizeof(double));

		address_block_info_map_.insert(
				std::make_pair(para_ptr, para_info)
		);

	}

	for (auto &para_ptr:parameter_block_vec) {
		auto &info = address_block_info_map_.find(para_ptr)->second;
		if (!info.removed_flag) {
			info.block_idx += remove_index;
		}
	}

	std::cout << "totally parameter:" << parameter_block_vec.size()
	          << " removed parameter:" << removed_block_num << std::endl;
	std::cout << "remain index:" << remain_index
	          << "remove index:" << remove_index
	          << "total index:" << total_index << std::endl;

	// generate jacobian matrix. and A, b.
	Eigen::MatrixXd A(total_index, total_index);
	Eigen::VectorXd b(total_index);

	/// for each residual block
	std::vector<ceres::ResidualBlockId> residual_id_vec;
	problem.GetResidualBlocks(&residual_id_vec);
	for (auto &residual_block_id:residual_id_vec) {
		auto *loss_ptr = problem.GetLossFunctionForResidualBlock(residual_block_id);
		auto *cost_ptr = problem.GetCostFunctionForResidualBlock(residual_block_id);

		auto &para_size_vec = cost_ptr->parameter_block_sizes();

		std::vector<double *> para_vec;
		problem.GetParameterBlocksForResidualBlock(residual_block_id,
		                                           &para_vec);
		Eigen::VectorXd residual_vector;
		residual_vector.resize(cost_ptr->num_residuals());

		std::vector<
				Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>
		> jaco_matrix_vec(para_vec.size());
		std::vector<double *> raw_jaco_matrix_vec(para_vec.size());
		for (int i = 0; i < para_vec.size(); ++i) {
			jaco_matrix_vec[i].resize(cost_ptr->num_residuals(), para_size_vec[i]);
			raw_jaco_matrix_vec[i] = jaco_matrix_vec[i].data();
		}

		cost_ptr->Evaluate(para_vec.data(),
		                   residual_vector.data(),
		                   raw_jaco_matrix_vec.data()
		);
		if (loss_ptr) {
			double residual_scaling_, alpha_sq_norm_;

			double sq_norm, rho[3];

			sq_norm = residual_vector.squaredNorm();
			loss_ptr->Evaluate(sq_norm, rho);

			double sqrt_rho1_ = std::sqrt(rho[1]);

			if ((std::abs(sq_norm) < 1e-7) || (rho[2] <= 0.0)) {
				residual_scaling_ = sqrt_rho1_;
				alpha_sq_norm_ = 0.0;
			} else {
				const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
				const double alpha = 1.0 - sqrt(D);
				residual_scaling_ = sqrt_rho1_ / (1.0 - alpha);
				alpha_sq_norm_ = alpha / sq_norm;
			}

			for (int i = 0;
			     i < static_cast<int>(para_vec.size());
			     ++i) {

				jaco_matrix_vec[i] =
						sqrt_rho1_ * (jaco_matrix_vec[i] - alpha_sq_norm_ * residual_vector
						                                   * (residual_vector.transpose() * jaco_matrix_vec[i]));

			}
			residual_vector *= residual_scaling_;
		}
		// calculate block for A and b.
		for (int i = 0; i < para_vec.size(); ++i) {
			int idx_i = address_block_info_map_[para_vec[i]].block_idx;
			int size_i = para_size_vec[i];
			Eigen::MatrixXd &jacobian_i = jaco_matrix_vec[i];
			for (int j = i; j < static_cast<int>(para_vec.size());
			     ++j) {
				int idx_j = address_block_info_map_[para_vec[j]].block_idx;
				int size_j = para_size_vec[j];
				Eigen::MatrixXd &jacobian_j = jaco_matrix_vec[j];
				if (i == j) {
					A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;

				} else {
					A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() *
					                                         jacobian_j;
					A.block(idx_j, idx_i, size_j, size_i) =
							A.block(idx_i, idx_j, size_i, size_j).transpose();
				}
				b.segment(idx_i, size_i) += jacobian_i.transpose() * residual_vector;
			}
		}
	}
	std::cout << "A size:" << A.rows() << "x" << A.cols()
	          << " b size:" << b.rows() << "x" << b.cols() << std::endl;


	// generate marginalization factor adopted information.
	Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, remove_index, remove_index) +
	                             A.block(0, 0, remove_index, remove_index).transpose());
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> svdA(Amm);


	Eigen::MatrixXd Amm_inv = svdA.eigenvectors() *
	                          Eigen::VectorXd((svdA.eigenvalues().array() > eps).select(
			                          svdA.eigenvalues().array().inverse(), 0)).asDiagonal()
	                          * svdA.eigenvectors().transpose();

	//
	//A = [ Amm Amr]
	//    [ Arm Arr]
	Eigen::MatrixXd Amr = A.block(0, remove_index, remove_index, remain_index);
	Eigen::MatrixXd Arm = A.block(remove_index, 0, remain_index, remove_index);
	Eigen::MatrixXd Arr = A.block(remove_index, remove_index, remain_index, remain_index);


	// b= [bmm]
	//    [brr]
	Eigen::VectorXd bmm = b.segment(0, remove_index);
	Eigen::VectorXd brr = b.segment(remove_index, remain_index);

	Eigen::MatrixXd Amarg = Arr - Arm * Amm_inv * Amr;
	Eigen::VectorXd bmarg = brr - Arm * Amm_inv * bmm;

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> svdAmarg(Amarg);
	Eigen::VectorXd S = Eigen::VectorXd(
			(svdAmarg.eigenvalues().array() > eps).
					select(svdAmarg.eigenvalues().array(), 0)
	);
	Eigen::VectorXd S_inv = Eigen::VectorXd(
			(svdAmarg.eigenvalues().array() > eps).
					select(
					svdAmarg.eigenvalues().array().inverse(), 0
			)
	);
	Eigen::VectorXd S_sqrt = S.cwiseSqrt();
	Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

	block_linearized_jac = S_sqrt.asDiagonal() * svdAmarg.eigenvectors().transpose();
	block_linear_residual = S_inv_sqrt.asDiagonal() * svdAmarg.eigenvectors().transpose() * bmarg;

	std::cout << "block jac size:" << block_linearized_jac.rows() << "x" << block_linearized_jac.cols()
	          << " block residual:" << block_linear_residual.rows() << "x" << block_linear_residual.cols()
	          << std::endl;

	// clear removed_block_set_
	removed_block_set_.clear();

	with_marginalization_info_flag_ = true;


	return true;
}


bool MarginalizationServer::InsertMarignalizationFactor(ceres::Problem &problem) {
	if (with_marginalization_info_flag_) {


		address_block_info_map_.clear();
		with_marginalization_info_flag_ = false;
		return true;
	} else {

		return false;
	}
}


bool MarginalizationServer::withMarginalizationInfo() const {
	return with_marginalization_info_flag_;
}


