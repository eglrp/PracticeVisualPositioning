//
// Created by steve on 3/30/19.
//

#ifndef PRACTICEVISUALPOSITIONING_CAMERAPROJECT_H
#define PRACTICEVISUALPOSITIONING_CAMERAPROJECT_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "omp.h"

class CameraProject {
public:
	/**
	 * @brief
	 */
	CameraProject() {
		CameraProject(300.0, 1280, 720);
	}

	/**
	 * @brief
	 * @param f
	 * @param height
	 * @param width
	 * @param cx
	 * @param cy
	 */
	CameraProject(double f,
	              int height = -1,
	              int width = -1,
	              double cx = -1.0,
	              double cy = -1.0) {

		CameraProject(f,
		              f,
		              height,
		              width,
		              cx,
		              cy);

	}


	/**
	 * @brief
	 * @param fx
	 * @param fy
	 * @param height
	 * @param width
	 * @param cx
	 * @param cy
	 */
	CameraProject(double fx,
	              double fy,
	              int height = -1,
	              int width = -1,
	              double cx = -1.0,
	              double cy = -1.0) {
		fx_ = fx;
		fy_ = fy;
		if (height > 0 and width > 0) {
			width_ = width;
			height_ = height;
			if (cx < 0.0 or cy < 0.0) {
				cx = double(width) / 2.0;
				cy = double(height) / 2.0;
			}
		}

		if (cx > 0.0 and cy > 0.0) {
			cx_ = cx;
			cy_ = cy;
		}
	}

	bool setBody2Cam(Eigen::Quaterniond qua_b2c,
	                 Eigen::Vector3d t_b2c) {
		qua_bc = Eigen::Quaterniond(qua_b2c);
		qua_bc = qua_b2c.normalized();
		t_bc = Eigen::Vector3d(t_b2c(0), t_b2c(1), t_b2c(2));
		return true;
	}


	bool projectToimage(
			const Eigen::Quaterniond &qua,
			const Eigen::Vector3d &t,
			const Eigen::MatrixXd &pts3d,
			Eigen::MatrixXd &pts_cam
	) {

#pragma omp parallel for
		for (int i = 0; i < pts3d.rows(); ++i) {
			pts_cam(i, 2) = -1.0;
			pts_cam.block<1, 3>(i, 0) = projectOnePoint(
					qua, t, pts3d.block<1, 3>(i, 0)
			);
		}
		return true;


	}

	Eigen::Vector3d projectOnePoint(
			Eigen::Quaterniond qua,
			Eigen::Vector3d t,
			Eigen::Vector3d pt
	) {
		Eigen::Vector3d p_cam(0, 0, 1.0);
		p_cam = qua_bc.toRotationMatrix() * (qua.toRotationMatrix().inverse() * (pt - t)) + t_bc;
		if (p_cam(2) < 1.0) {
			return Eigen::Vector3d(0, 0, -1.0);
		}
		p_cam(0) = p_cam(0) / p_cam(2);
		p_cam(1) = p_cam(1) / p_cam(2);

		p_cam(0) = p_cam(0) * fx_ + cx_;
		p_cam(1) = p_cam(1) * fy_ + cy_;
//		std::cout << p_cam.transpose() << std::endl;
		if (p_cam(0) > 0 && p_cam(0) < width_ && p_cam(1) > 0 && p_cam(1) < height_) {

			return p_cam;
		} else {
			return Eigen::Vector3d(0, 0, -1.0);
		}

	}

//protected:
	// camera matrix
	double fx_ = 1.0;
	double fy_ = 1.0;
	double cx_ = 1.0;
	double cy_ = 1.0;

	// image size
	int width_ = 720;
	int height_ = 480;

	Eigen::Quaterniond qua_bc = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
	Eigen::Vector3d t_bc = Eigen::Vector3d(0.0, 0.0, 0.0);


};


#endif //PRACTICEVISUALPOSITIONING_CAMERAPROJECT_H
