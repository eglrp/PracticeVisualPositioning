//
// Created by steve on 11/15/18.
//

#ifndef BASESLAM_FRAME_H
#define BASESLAM_FRAME_H

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/line_descriptor.hpp>

#include <VisualOdometry/StereoCamera.h>


#include <iostream>
#include <memory>


#include <eigen3/Eigen/Geometry>

//using namespace gtsam;

namespace BaseSLAM {
	class Frame {
	public:
//		std::shared_ptr<BaseSLAM::StereoCamera> cam_ptr_; // camera model

		long frame_id_;// id of frame
		double time_stampe_; // when it is recorded

		Eigen::Isometry3d transform_matrix_; // transform matrix  form world to camera

		std::vector<double> time_vec_;
		std::vector<Eigen::Vector3d> acc_vec_;
		std::vector<Eigen::Vector3d> gyr_vec_;
		std::vector<Eigen::Vector3d> mag_vec_;

		std::vector<int> contained_feature_id_;


	};


}


#endif //BASESLAM_FRAME_H
