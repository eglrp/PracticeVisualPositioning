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

	int cam_id_offset = 7000000;
	int feature_offset= 6000000;

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

	//create g2o camera para

//	g2o::ParameterCamera *camera_parameter = new g2o::ParameterCamera();
//	camera_parameter->setId(cam_id_offset + 1);
//	printf("set camera parameter fx:%f,fy:%f,cx:%f,cy:%f\n",
//	       left_cam_matrix.at<float>(0, 1),
//	       left_cam_matrix.at<float>(1, 1),
//	       left_cam_matrix.at<float>(0, 2),
//	       left_cam_matrix.at<float>(1, 2));
//	camera_parameter->setKcam(
//			left_cam_matrix.at<float>(0, 1),
//			left_cam_matrix.at<float>(1, 1),
//			left_cam_matrix.at<float>(0, 2),
//			left_cam_matrix.at<float>(1, 2));
//	int cam_param_index = 900000000;
//	camera_parameter->setId(cam_param_index);

	for(int i=0;i<img_reader.vec_size_;++i){
		whole_img = img_reader.get_image(i);
		std::cout << " readed " << i << "-th image" << std::endl;


		left_img = img_reader.copy_left_img(whole_img);
		right_img = img_reader.copy_right_img(whole_img);

		cv::cvtColor(left_img,left_img,cv::COLOR_BGR2GRAY);

//		if(i%3==0)
		featureTrackServer.addNewFrame(left_img);
		std::cout << featureTrackServer.cur_frame_id_ << std::endl;


		std::vector<cv::Point2f> un_pts;
		cv::undistortPoints(featureTrackServer.cur_pts_,un_pts,
				stereo_camera_ptr->M1,stereo_camera_ptr->D1);
		g2o::VertexSE3 * current_cam_vertex = new g2o::VertexSE3();
		current_cam_vertex->setId(featureTrackServer.cur_frame_id_);
		Eigen::Isometry3d ini_pos = Eigen::Isometry3d::Identity();
		ini_pos.matrix()(0,3) = double(featureTrackServer.curr_feature_id_)/1000.0;
		ini_pos.matrix()(0,1) = double(featureTrackServer.curr_feature_id_)/1000.0;
		current_cam_vertex->setEstimate(Eigen::Isometry3d::Identity());
		if(featureTrackServer.cur_frame_id_==1){
			current_cam_vertex->setFixed(true);
		}
		globalOptimizer.addVertex(current_cam_vertex);

		Eigen::Matrix3d points_info = Eigen::Matrix3d::Identity() * 50.0;

		for(int fi=0;fi < un_pts.size();++fi){
			int feature_id = featureTrackServer.ids_[fi];
			std::cout << "here feature id:"<< feature_id << std::endl;
			if(feature_id>0&&featureTrackServer.track_cnt_[fi]>3) {
//				std::cout << "track cnt:" << featureTrackServer.track_cnt_[fi] << std::endl;
				if (featureTrackServer.track_cnt_[fi] == 4) {
//				try{
					g2o::VertexPointXYZ *pointXyz = new g2o::VertexPointXYZ();
					pointXyz->setId(feature_id + feature_offset);
					pointXyz->setEstimate(Eigen::Vector3d(2.0, 3.0, 0.0));
					globalOptimizer.addVertex(pointXyz);
//				}

				}

				g2o::EdgeSE3PointXYZDisparity *ob_pt = new g2o::EdgeSE3PointXYZDisparity();
				ob_pt->vertices()[0] = globalOptimizer.vertex(featureTrackServer.cur_frame_id_);
				ob_pt->vertices()[1] = globalOptimizer.vertex(feature_id + feature_offset);
				ob_pt->setParameterId(0, cam_param_index);

				ob_pt->setMeasurement(Eigen::Vector3d(
						un_pts[fi].x,
						un_pts[fi].y,
						0.1
				));

				ob_pt->setInformation(points_info);

				globalOptimizer.addEdge(ob_pt);
			}

		}



		cv::imshow("left_src",left_img);
		cv::imshow("right_src", right_img);

		if(i%20==0&& i <300){
			globalOptimizer.initializeOptimization();
			globalOptimizer.optimize();
		}
		cv::waitKey(10);

	}

	globalOptimizer.save("/home/steve/test_hug.g2o");

	cv::waitKey();




}