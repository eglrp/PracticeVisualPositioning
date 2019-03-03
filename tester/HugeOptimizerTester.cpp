//
// Created by steve on 3/3/19.
//

#include <iostream>

#include <opencv2/opencv.hpp>
#include <VisualOdometry/StereoCamera.h>


#include "VisualOdometry/FeatureTrackServer.h"
#include "VisualOdometry/FeatureTrackServer.cpp"

#include "util/MYNTEYEReader.h"


#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"


#include "g2o/types/sba/sbacam.h"
#include "g2o/types/sba/types_sba.h"
#include "g2o/types/sba/g2o_types_sba_api.h"

#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"

int main(){
	auto *stereo_camera_ptr = new BaseSLAM::StereoCamera("/home/steve/Data/MYNTVI/camera_parameter1.yaml");
	stereo_camera_ptr->print("camera");
//	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco0012.list");
//	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco006.list");
//	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco-hard1.list");
	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco-hard1.list");

	cv::Mat whole_img, left_img,right_img;

	FeatureTrackServer featureTrackServer;
	featureTrackServer.setCameraParameter(stereo_camera_ptr->M1,stereo_camera_ptr->D1);

	/**
	 * @brief g2o part
	 */
	g2o::SparseOptimizer globalOptimizer;

	typedef g2o::BlockSolverX SlamBlockSolver;
	typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

	// 初始化求解器
	// create the linear solver
	auto linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();

	// create the block solver on top of the linear solver
	auto blockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));

	g2o::OptimizationAlgorithmLevenberg *optimizationAlgorithm = new g2o::OptimizationAlgorithmLevenberg(
			std::move(blockSolver));


	globalOptimizer.setAlgorithm(optimizationAlgorithm);


	g2o::RobustKernel *rbk = g2o::RobustKernelFactory::instance()->construct("Cauchy");
	rbk->setDelta(0.2);

	auto left_cam_matrix = stereo_camera_ptr->M1;

	int cam_id_offset = 5000000;
	g2o::ParameterCamera *camera_parameter = new g2o::ParameterCamera();
	camera_parameter->setId(cam_id_offset + 1);
	printf("set camera parameter fx:%f,fy:%f,cx:%f,cy:%f\n",
	       left_cam_matrix.at<float>(0, 1),
	       left_cam_matrix.at<float>(1, 1),
	       left_cam_matrix.at<float>(0, 2),
	       left_cam_matrix.at<float>(1, 2));
	camera_parameter->setKcam(
			left_cam_matrix.at<float>(0, 1),
			left_cam_matrix.at<float>(1, 1),
			left_cam_matrix.at<float>(0, 2),
			left_cam_matrix.at<float>(1, 2));
	int cam_param_index = 900000000;
	camera_parameter->setId(cam_param_index);
	globalOptimizer.addParameter(camera_parameter);


	std::map<int,std::vector<int>> feature_frames_map;



	for(int i=0;i<img_reader.vec_size_;++i){
		whole_img = img_reader.get_image(i);
		std::cout << " readed " << i << "-th image" << std::endl;


		left_img = img_reader.copy_left_img(whole_img);
		right_img = img_reader.copy_right_img(whole_img);

		cv::cvtColor(left_img,left_img,cv::COLOR_BGR2GRAY);

//		if(i%3==0)
		featureTrackServer.addNewFrame(left_img);
		std::cout << featureTrackServer.cur_frame_id_ << std::endl;

		for(int fi=0;fi < featureTrackServer.cur_pts_.size();++fi){


		}



		cv::imshow("left_src",left_img);
		cv::imshow("right_src", right_img);

		cv::waitKey(10);

	}

	cv::waitKey();




}