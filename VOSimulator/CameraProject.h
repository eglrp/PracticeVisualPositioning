//
// Created by steve on 3/30/19.
//

#ifndef PRACTICEVISUALPOSITIONING_CAMERAPROJECT_H
#define PRACTICEVISUALPOSITIONING_CAMERAPROJECT_H

#include <Eigen/Dense>
#include <Eigen/Geometry>


class CameraProject {
public:
	CameraProject() {}

	CameraProject(double f, double cx, double cy) {
		fx_ = f;
		fy_ = f;
		cx_ = cx;
		cy_ = cy;
	}

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
