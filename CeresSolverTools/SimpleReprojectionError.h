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
			double dx,
			double obx,
			double oby) :
			fx_(fx), dx_(dx), fy_(fy), cx_(cx), cy_(cy), obx_(obx), oby_(oby) {
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
		p[0] += t[0];
		p[0] += T(dx_);
		p[1] += t[1];
		p[2] += t[2];
		std::cout << "p[0]:" << p[0]
		          << "p[1]:" << p[1]
		          << "p[2]:" << p[2] << std::endl;


		T xp = p[0] / p[2];
		T yp = p[1] / p[2];



		if (!ceres::IsNormal(xp) || !ceres::IsNormal(yp)) {
//			std::cout << "pt:" << pt[0] << "," << pt[1] << "," << pt[1] << std::endl;
//			std::cout << "q:" << q[0] << "," << q[1] << "," << q[2] << "," << q[3] << std::endl;
//			std::cout << "t:"  << t[0] << "," << t[1] << "," << t[2] << std::endl;
//			std::cout << "SOME ERROR HAPPEND:!" << std::endl;

		}else{
//			xp = p[0];
//			yp = p[1];
		}
		std::cout << "xp:" << xp << "yp:" << yp << std::endl;

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
			double dx,
			double obx,
			double oby) {
		return (new ceres::AutoDiffCostFunction<SimpleReprojectionError, 2, 3, 4, 3>(
				new SimpleReprojectionError(fx, fy, cx, cy, dx, obx, oby)
		));
	}


	double fx_, fy_, cx_, cy_, obx_, oby_, dx_;
};


#endif //PRACTICEVISUALPOSITIONING_SIMPLEREPROJECTIONERROR_H
