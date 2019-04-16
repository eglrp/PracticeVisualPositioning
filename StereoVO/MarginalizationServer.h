//
// Created by steve on 4/11/19.
//

#ifndef PRACTICEVISUALPOSITIONING_MARGINALIZATIONSERVER_H
#define PRACTICEVISUALPOSITIONING_MARGINALIZATIONSERVER_H

#include <ceres/ceres.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

struct ParameterBlockInfo {
	const double *block_para_address = nullptr;
	int block_size = -1;
	int block_idx = -1;
	bool removed_flag = false;

	double *keeped_block_value = nullptr;

	~ParameterBlockInfo() {
		delete[] keeped_block_value;
	}
};

struct ResidualBlockInfo {
	ceres::CostFunction *cost_func_ptr;
	std::vector<double *> para_block_vec;

};

class MarginalizationServer {
public:
	MarginalizationServer() {

	}


	bool AddResidualInfo(ceres::CostFunction *func_ptr,
	                     std::vector<double *> para_ptr_vec);

	bool AddInvDepthResidualInfo(ceres::CostFunction *cost_func_ptr,
	                             double *pre_qua_ptr, double *pre_pos_ptr,
	                             double *qua_ptr, double *pos_ptr, double *inv_depth_ptr);

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
	std::map<double *, ParameterBlockInfo> address_block_info_map_; // recored relate information here.
	// save the information adopted in marginalization constraint
	Eigen::MatrixXd block_linearized_jac;
	Eigen::MatrixXd block_linear_residual;


	// Save information temp.
	std::set<double *> removed_block_set_;
//	std::vector<ceres::CostFunction *> cost_func_vec;
	std::vector<ResidualBlockInfo> residual_block_vec;


};


#endif //PRACTICEVISUALPOSITIONING_MARGINALIZATIONSERVER_H
