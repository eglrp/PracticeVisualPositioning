//
// Created by steve on 4/11/19.
//

#ifndef PRACTICEVISUALPOSITIONING_MARGINALIZATIONSERVER_H
#define PRACTICEVISUALPOSITIONING_MARGINALIZATIONSERVER_H

#include <ceres/ceres.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

struct BlockInfo {
	const double *block_para_address = nullptr;
	int block_size = -1;
	int block_idx = -1;
	bool removed_flag = false;

	// save the information adopted in marginalization constraint
	Eigen::MatrixXd block_linearized_jac;
	Eigen::MatrixXd block_linear_residual;
	Eigen::MatrixXd keeped_block_value;
};

class MarginalizationServer {
public:
	MarginalizationServer() {

	}




	bool AddCostFunction(ceres::CostFunction *func_ptr);

	/**
	 * @brief save removed paramteres to
	 * @param para_ptr
	 * @return
	 */
	bool markRemovedParameter(double *para_ptr);

	bool MarignalizationProcess();

	bool InsertMarignalizationFactor(ceres::Problem &problem);

	// return state of marginalizationg
	bool withMarginalizationInfo() const;


protected:
	bool with_marginalization_info_flag_ = false;


	// save marginalization preprocessed data.
	std::map<double *, BlockInfo> address_block_info_map_; // recored relate information here.


	// Save information temp.
	std::set<double *> removed_block_set_;
	std::vector<ceres::CostFunction *> cost_func_vec;


};


#endif //PRACTICEVISUALPOSITIONING_MARGINALIZATIONSERVER_H
