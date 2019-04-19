//
// Created by steve on 4/11/19.
//

#ifndef PRACTICEVISUALPOSITIONING_MARGINALIZATIONSERVER_H
#define PRACTICEVISUALPOSITIONING_MARGINALIZATIONSERVER_H

#include <ceres/ceres.h>

#include <CeresSolverTools/MarginalizationFactor.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <StereoVO/MarginalizationInfo.h>
#include <omp.h>

#include <util/tic_toc.h>

class MarginalizationServer {
public:
	MarginalizationServer() {

	}


	/**
	 * @brief save removed paramteres to
	 * @param para_ptr
	 * @return
	 */
	bool markRemovedParameter(double *para_ptr);

	/**
	 * @brief
	 * @return
	 */
	bool MarignalizationProcess(ceres::Problem &problem);

	bool InsertMarignalizationFactor(ceres::Problem &problem);


	// return state of marginalizationg
	bool withMarginalizationInfo() const;


protected:
	bool with_marginalization_info_flag_ = false;

	double eps = 1e-5;

	// save marginalization preprocessed data.
	// recored relate information here.
	std::map<double *, ParameterBlockInfo> address_block_info_map_;
	// save the information adopted in marginalization constraint
	Eigen::MatrixXd block_linearized_jac;
	Eigen::MatrixXd block_linear_residual;

	std::vector<double *> remain_sorted_vec_;


	// Save information temp.
	std::set<double *> removed_block_set_;

	// temperary struct


};


#endif //PRACTICEVISUALPOSITIONING_MARGINALIZATIONSERVER_H
