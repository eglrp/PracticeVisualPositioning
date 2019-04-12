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

bool MarginalizationServer::MarignalizationProcess() {

}

