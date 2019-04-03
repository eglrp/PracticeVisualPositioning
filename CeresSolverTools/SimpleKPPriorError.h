//
// Created by steve on 4/2/19.
//

#ifndef PRACTICEVISUALPOSITIONING_SIMPLEKPPRIORERROR_H
#define PRACTICEVISUALPOSITIONING_SIMPLEKPPRIORERROR_H

#include <ceres/ceres.h>

struct SimpleKPPriorError {
	SimpleKPPriorError(
			double px,
			double py,
			double pz,
			double cov_x,
			double cov_y,
			double cov_z
			):px_(px),py_(py),pz_(pz),cov_x_(cov_x),cov_y_(cov_y),cov_z_(cov_z)
	{


	}

	template <typename T>
	bool operator()(
			const T *const pt,
			T *residuals
			)const {
		residuals[0] = (pt[0] - T(px_)) / cov_x_;
		residuals[1] = (pt[1] - T(py_)) / cov_y_;
		residuals[2] = (pt[2] - T(pz_)) / cov_z_;
		return true;
	}

	static ceres::CostFunction * Create(
			double px,double py,double pz,
			double cx, double cy, double cz
			){
		return (new ceres::AutoDiffCostFunction<SimpleKPPriorError,3,3>(
				new SimpleKPPriorError(px,py,pz,cx,cy,cz)
				));
	}


	double px_,py_,pz_,cov_x_,cov_y_,cov_z_;




};


#endif //PRACTICEVISUALPOSITIONING_SIMPLEKPPRIORERROR_H
