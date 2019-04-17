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


bool MarginalizationServer::MarignalizationProcess() {
	// initial parameter idx.



	// generate jacobian matrix. and A, b.



	//



	return true;
}


bool MarginalizationServer::InsertMarignalizationFactor(ceres::Problem &problem) {


}


bool MarginalizationServer::withMarginalizationInfo() const {
	return with_marginalization_info_flag_;
}

bool MarginalizationServer::AddResidualInfo(ceres::CostFunction *func_ptr,
                                            std::vector<double *> para_ptr_vec) {

	// para ptr vec size should equal to the number of parametere block in cost function.
	if (func_ptr->parameter_block_sizes().size() == para_ptr_vec.size()) {
		ResidualBlockInfo residualBlockInfo;
		residualBlockInfo.cost_func_ptr = func_ptr;
		residualBlockInfo.para_block_vec = para_ptr_vec;

		residual_block_vec.push_back(residualBlockInfo);

		return true;
	} else {
		return false;
	}
}

bool MarginalizationServer::AddInvDepthResidualInfo(ceres::CostFunction *cost_func_ptr, double *pre_qua_ptr,
                                                    double *pre_pos_ptr, double *qua_ptr, double *pos_ptr,
                                                    double *inv_depth_ptr) {
	std::vector<double *> para_ptr_vec;
	para_ptr_vec.push_back(pre_qua_ptr);
	para_ptr_vec.push_back(pre_pos_ptr);
	para_ptr_vec.push_back(qua_ptr);
	para_ptr_vec.push_back(pos_ptr);
	para_ptr_vec.push_back(inv_depth_ptr);
	return AddResidualInfo(cost_func_ptr, para_ptr_vec);
}
