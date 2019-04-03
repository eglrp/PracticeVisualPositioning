//
// Created by steve on 4/3/19.
//

#ifndef PRACTICEVISUALPOSITIONING_SIMPLEINVDEPTHREPROJECTIONERROR_H
#define PRACTICEVISUALPOSITIONING_SIMPLEINVDEPTHREPROJECTIONERROR_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct SimpleInvDepthReProjectionError {


	double obx_i_,oby_i_,obx_j_,oby_j_;

	double sqrt_info_;// a better implement is 2x2 matrix rather than a value.

//	double* q_bc_i_,t_bc_i_; // body to camera projection.// set boty to camera transfrorm at i-th moment is identity.
	double* q_bc_j_,t_bc_j_; // body to camera projection.

};


#endif //PRACTICEVISUALPOSITIONING_SIMPLEINVDEPTHREPROJECTIONERROR_H
