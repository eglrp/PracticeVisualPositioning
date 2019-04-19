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
	remain_sorted_vec_.clear();
	remain_sorted_size_vec_.clear();
	address_block_info_map_.clear();


	// initial parameter idx.
	int remain_index = 0;
	int remove_index = 0;
	int total_index = 0;


	// recored all parameter block
	std::vector<double *> parameter_block_vec;
	std::vector<int> parameter_size_vec;
	problem.GetParameterBlocks(&parameter_block_vec);


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
			remain_sorted_vec_.push_back(para_ptr);
			remain_sorted_size_vec_.push_back(block_size);
		}

		para_info.keeped_block_value = Eigen::VectorXd(block_size);
		memcpy(para_info.keeped_block_value.data(), para_ptr, block_size * sizeof(double));

		address_block_info_map_.insert(
				std::make_pair(para_ptr, para_info)
		);

	}
	total_index = remain_index + remove_index + 1;
	for (auto &para_ptr:parameter_block_vec) {
		auto &info = address_block_info_map_.find(para_ptr)->second;
		if (!info.removed_flag) {
			info.total_idx = info.block_idx + remove_index;
		} else {
			info.total_idx = info.block_idx;
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

	A.setZero();
	b.setZero();

	/// for each residual block
	std::vector<ceres::ResidualBlockId> residual_id_vec;
	problem.GetResidualBlocks(&residual_id_vec);
//	for (auto &residual_block_id:residual_id_vec) {
	std::cout << "total residual block number :" << residual_id_vec.size() << std::endl;
//	omp_set_num_threads(12);
	TicToc tt;

	int jac_num_thread = 12;
	std::vector<Eigen::MatrixXd> local_A_vec;
	std::vector<Eigen::VectorXd> local_b_vec;
	for (int i = 0; i < jac_num_thread; ++i) {
		local_A_vec.push_back(Eigen::MatrixXd(A.rows(), A.cols()));
		local_A_vec[i].setZero();
		local_b_vec.push_back(Eigen::VectorXd(b.rows()));
		local_b_vec[i].setZero();
	}


#pragma omp parallel for
	for (int index = 0; index < residual_id_vec.size(); ++index) {
		auto &residual_block_id = residual_id_vec[index];
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

				if (!std::isfinite(jaco_matrix_vec[i].sum())) {
					std::cout << "before adopting loss func jacobian matrix error:"
					          << jaco_matrix_vec[i] << "\n idx:"
					          << i << std::endl;
				}
				jaco_matrix_vec[i] =
						sqrt_rho1_ * (jaco_matrix_vec[i] - alpha_sq_norm_ * residual_vector
						                                   * (residual_vector.transpose() * jaco_matrix_vec[i]));
				if (!std::isfinite(jaco_matrix_vec[i].sum())) {
					std::cout << "after adopting loss func jacobian matrix error:"
					          << jaco_matrix_vec[i] << "\n idx:"
					          << i << std::endl;
				}

			}
			residual_vector *= residual_scaling_;
		}



//			std::cout << "cur thread:" <<omp_get_thread_num()
//			<< " total threads:" << omp_get_num_threads()<< std::endl;
		// calculate block for A and b.
//#pragma omp parallel for
		int thread_index = omp_get_thread_num();
		assert(thread_index < jac_num_thread);
		for (int i = 0; i < para_vec.size(); ++i) {
			int idx_i = address_block_info_map_[para_vec[i]].block_idx;
			int size_i = para_size_vec[i];
			Eigen::MatrixXd &jacobian_i = jaco_matrix_vec[i];
			for (int j = i; j < static_cast<int>(para_vec.size());
			     ++j) {
				int idx_j = address_block_info_map_[para_vec[j]].block_idx;
				int size_j = para_size_vec[j];
				Eigen::MatrixXd &jacobian_j = jaco_matrix_vec[j];
				Eigen::MatrixXd iTj = jacobian_i.transpose() * jacobian_j;
				// data for each element.
				if (i == j) {
					local_A_vec[thread_index].block(idx_i, idx_j, size_i, size_j) += iTj;

				} else {
					local_A_vec[thread_index].block(idx_i, idx_j, size_i, size_j) += iTj;
					local_A_vec[thread_index].block(idx_j, idx_i, size_j, size_i) += iTj.transpose();
//								A.block(idx_i, idx_j, size_i, size_j).transpose().eval();
				}


				local_b_vec[thread_index].segment(idx_i, size_i) += jacobian_i.transpose() * residual_vector;
			}

		}
	}

	for (int i = 0; i < local_A_vec.size(); ++i) {
		A += local_A_vec[i];
		b += local_b_vec[i];
	}


	std::cout << "A size:" << A.rows() << "x" << A.cols()
	          << " b size:" << b.rows() << "x" << b.cols() << std::endl;
	std::cout << "calculate using time :" << tt.toc() << std::endl;


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
//	Eigen::isfinite(block_linearized_jac)

	// clear removed_block_set_
	removed_block_set_.clear();

	with_marginalization_info_flag_ = true;


	return true;
}


bool MarginalizationServer::InsertMarignalizationFactor(ceres::Problem &problem) {
	if (with_marginalization_info_flag_) {

		ceres::CostFunction *marg_cost_func_ptr = new MarginalizationFactor(
				&address_block_info_map_,
				remain_sorted_size_vec_,
				block_linearized_jac,
				block_linear_residual
		);
		std::cout << "remain size:" << remain_sorted_vec_.size()
		          << " cost func para block number:" << marg_cost_func_ptr->parameter_block_sizes().size() << std::endl;

		problem.AddResidualBlock(
				marg_cost_func_ptr,
				NULL,
				remain_sorted_vec_
		);


		with_marginalization_info_flag_ = false;
		return true;
	} else {

		return false;
	}
}


bool MarginalizationServer::withMarginalizationInfo() const {
	return with_marginalization_info_flag_;
}


