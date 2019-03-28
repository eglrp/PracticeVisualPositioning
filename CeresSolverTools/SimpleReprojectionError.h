//
// Created by steve on 3/27/19.
//

#ifndef PRACTICEVISUALPOSITIONING_SIMPLEREPROJECTIONERROR_H
#define PRACTICEVISUALPOSITIONING_SIMPLEREPROJECTIONERROR_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>

class SimpleReprojectionError {
	SimpleReprojectionError(
			double fx,
			double fy,
			double cx,
			double cy,
			double obx,
			double oby) :
			fx_(fx), fy_(fy), cx_(cx), cy_(cy), obx_(obx), oby_(oby) {
	}

	template<typename T>
	bool operator()(
			const T *const pt,
			const T *const q,
			const T *const t,
			T *residuals
	) const {
		T p[3];
		ceres::QuaternionRotatePoint(q, pt, p);
		p[0] += T(t[0]);
		p[1] += T(t[1]);
		p[2] += T(t[2]);

		T xp = p[0] / p[2];
		T yp = p[1] / p[2];

		T fx = T(fx_);
		T fy = T(fy_);
		T cx = T(cx_);
		T cy = T(cy_);

		T pre_x = fx * xp + cx;
		T pre_y = fy * yp + cy;

		residuals[0] = pre_x - T(obx_);
		residuals[1] = pre_y - T(oby_);


	}

	static ceres::CostFunction *Create(
			double fx,
			double fy,
			double cx,
			double cy,
			double obx,
			double oby) {
		return (new ceres::AutoDiffCostFunction<SimpleReprojectionError, 2, 3, 4, 3>(
				new SimpleReprojectionError(fx, fy, cx, cy, obx, oby)
		));
	}


	double fx_, fy_, cx_, cy_, obx_, oby_;
};


#endif //PRACTICEVISUALPOSITIONING_SIMPLEREPROJECTIONERROR_H
