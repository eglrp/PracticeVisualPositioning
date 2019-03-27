//
// Created by steve on 3/26/19.
//

#ifndef PRACTICEVISUALPOSITIONING_PROJECTIONFACTORCAM_H
#define PRACTICEVISUALPOSITIONING_PROJECTIONFACTORCAM_H

#include <ceres/ceres.h>
#include <Eigen/Dense>

class ProjectionFactorCam :public ceres::SizedCostFunction<2,7,7,1>{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	ProjectionFactorCam(const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j, Eigen::Matrix4d cTb);
	bool Evaluate( double const *const * parameters,
			double *residuals, double **jacobians) const;

	Eigen::Vector3d pts_i, pts_j;
	Eigen::Matrix3d cTb;
	Eigen::Matrix<double,2,3> tangent_base;
	static Eigen::Matrix2d sqrt_info;
	static double sum_t;


};
ProjectionFactorCam::ProjectionFactorCam(const Eigen::Vector3d &pts_i, const Eigen::Vector3d &pts_j,
                                         Eigen::Matrix4d cTb_mat){
	cTb  = cTb_mat;
	pts_i =



}


bool ProjectionFactorCam::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {

}


#endif //PRACTICEVISUALPOSITIONING_PROJECTIONFACTORCAM_H
