//
// Created by steve on 11/15/18.
//

#ifndef BASESLAM_FRAME_H
#define BASESLAM_FRAME_H

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>

#include <opencv2/line_descriptor.hpp>

#include <util/DataUnit.h>
#include <VisualOdometry/StereoCamera.h>


#include <iostream>
#include <memory>


//using namespace gtsam;

namespace BaseSLAM {
	class Frame {
	public:
		std::shared_ptr<BaseSLAM::StereoCamera> cam_ptr_; // camera model
//		std::shared_ptr<BaseSLAM::StereoINSData> data_ptr_;//
		BaseSLAM::StereoINSData *data_ptr_;//

		long idx_;// id of frame
		double time_stampe_; // when it is recorded

		Eigen::Isometry3d transform_matrix_; // transform matrix  form world to camera


		std::vector<cv::KeyPoint> left_feature_points_, right_feature_points_;// key points (or feature points)
		cv::Mat left_descriptors_, right_descriptors_;

		std::vector<cv::line_descriptor::KeyLine> left_lines_, right_lines_; // line descriptors.
		cv::Mat left_line_descriptors_, right_line_descriptors_;


		Frame(std::shared_ptr<StereoCamera> cam_ptr, StereoINSData *data_ptr, int index) {
			idx_ = index;
			cam_ptr_ = cam_ptr;
			data_ptr_ = data_ptr;
			time_stampe_ = data_ptr->global_timestamp_;

		}


	};


}


#endif //BASESLAM_FRAME_H
