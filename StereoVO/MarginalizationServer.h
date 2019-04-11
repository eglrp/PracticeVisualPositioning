//
// Created by steve on 4/11/19.
//

#ifndef PRACTICEVISUALPOSITIONING_MARGINALIZATIONSERVER_H
#define PRACTICEVISUALPOSITIONING_MARGINALIZATIONSERVER_H

#include <ceres/ceres.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

struct BlockInfo{
	int block_size= -1;

	// save the fi
	Eigen::MatrixXd block_linearized_jac;
	Eigen::MatrixXd block_linear_residual;
	Eigen::MatrixXd keeped_block_value;
};

class MarginalizationServer {
public:
	MarginalizationServer(){

	}


protected:
	std::map<double *,BlockInfo> address_block_info_map_;




};


#endif //PRACTICEVISUALPOSITIONING_MARGINALIZATIONSERVER_H
