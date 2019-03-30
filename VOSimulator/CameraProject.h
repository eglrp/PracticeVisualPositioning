//
// Created by steve on 3/30/19.
//

#ifndef PRACTICEVISUALPOSITIONING_CAMERAPROJECT_H
#define PRACTICEVISUALPOSITIONING_CAMERAPROJECT_H

#include <Eigen/Dense>
#include <Eigen/Geometry>


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


	bool projectToimage(
			Eigen::Quaterniond qua,
			Eigen::Vector3d t,
			Eigen::MatrixXd pts3d
	);

protected:
	// camera matrix
	double fx_ = 1.0;
	double fy_ = 1.0;
	double cx_ = 1.0;
	double cy_ = 1.0;

	// image size
	int width_ = 720;
	int height_ = 480;


};


#endif //PRACTICEVISUALPOSITIONING_CAMERAPROJECT_H
