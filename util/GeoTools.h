//
// Created by steve on 3/26/19.
//

#ifndef PRACTICEVISUALPOSITIONING_GEOTOOLS_H
#define PRACTICEVISUALPOSITIONING_GEOTOOLS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>


/**
 * @brief calculate 3d position of a point observed by two frame(0 and 1).
 * @param R0 rotation matrix of 0 moment
 * @param t0  translation vector of 0 moment.
 * @param R1
 * @param t1
 * @param pt0  points (2d)
 * @param pt1
 * @param pt3d 3D position of points.
 * @return
 */
bool triangulatePoint(Eigen::Matrix<double, 3,3> &R0,Eigen::Matrix<double,3,1> &t0,
		Eigen::Matrix<double, 3,3>&R1, Eigen::Matrix<double, 3,1> &t1,
		Eigen::Vector2d &pt0, Eigen::Vector2d &pt1,
		Eigen::Vector3d &pt3d){

}


#endif //PRACTICEVISUALPOSITIONING_GEOTOOLS_H
