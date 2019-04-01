//
// Created by steve on 3/27/19.
//

#ifndef PRACTICEVISUALPOSITIONING_SIMPLEREPROJECTIONERROR_H
#define PRACTICEVISUALPOSITIONING_SIMPLEREPROJECTIONERROR_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>

struct SimpleReprojectionError {
	SimpleReprojectionError(
			double fx,
			double fy,
			double cx,
			double cy,
			double obx,
			double oby,
			double *q_bc,
			double *t_bc) :
			fx_(fx), fy_(fy), cx_(cx), cy_(cy), obx_(obx), oby_(oby) {
		q_bc_ = q_bc;
		t_bc_ = t_bc;
	}

	template<typename T>
	bool operator()(
			const T *const pt,
			const T *const q,
			const T *const t,
			T *residuals
	) const {

		T p[3];
		p[0] = pt[0] - t[0];
		p[1] = pt[1] - t[1];
		p[2] = pt[2] - t[2];

		T pb[3];
		ceres::QuaternionRotatePoint(q, p, pb);
		T pc[3];

		T q_bc[4];
		q_bc[0] = T(q_bc_[0]);
		q_bc[1] = T(q_bc_[1]);
		q_bc[2] = T(q_bc_[2]);
		q_bc[3] = T(q_bc_[3]);

		T t_bc[3];
		t_bc[0] = T(t_bc_[0]);
		t_bc[1] = T(t_bc_[1]);
		t_bc[2] = T(t_bc_[2]);

		ceres::QuaternionRotatePoint(q_bc, pb, pc);
		pc[0] += t_bc[0];
		pc[1] += t_bc[1];
		pc[2] += t_bc[2];


		T xp = pc[0] / pc[2];
		T yp = pc[1] / pc[2];


		T fx = T(fx_);
		T fy = T(fy_);
		T cx = T(cx_);
		T cy = T(cy_);

		T pre_x = fx * xp + cx;
		T pre_y = fy * yp + cy;

		residuals[0] = pre_x - T(obx_);
		residuals[1] = pre_y - T(oby_);
		return true;


	}

	static ceres::CostFunction *Create(
			double fx,
			double fy,
			double cx,
			double cy,
			double obx,
			double oby,
			double *q_bc,
			double *t_bc) {
		return (new ceres::AutoDiffCostFunction<SimpleReprojectionError, 2, 3, 4, 3>(
				new SimpleReprojectionError(fx, fy, cx, cy, obx, oby, q_bc, q_bc)
		));
	}


	double fx_, fy_, cx_, cy_, obx_, oby_;
	double *q_bc_;
	double *t_bc_;
};


#endif //PRACTICEVISUALPOSITIONING_SIMPLEREPROJECTIONERROR_H
