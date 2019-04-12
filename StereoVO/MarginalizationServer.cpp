//
// Created by steve on 4/11/19.
//

#include "MarginalizationServer.h"

bool MarginalizationServer::AddCostFunction(ceres::CostFunction *func_ptr) {
	if (func_ptr != nullptr) {

		cost_func_vec.push_back(func_ptr);
		return true;
	} else {
		return false;
	}
}

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



	return true;
}


bool MarginalizationServer::InsertMarignalizationFactor(ceres::Problem &problem) {

}


bool MarginalizationServer::withMarginalizationInfo() const{
	return with_marginalization_info_flag_;
}
