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

#include "g2o/types/sba/types_six_dof_expmap.h"

#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"

int main() {

	std::ofstream out_file("/home/steve/ba_trace.csv");
	auto *stereo_camera_ptr = new BaseSLAM::StereoCamera("/home/steve/Data/MYNTVI/camera_parameter1.yaml");
	stereo_camera_ptr->print("camera");
//	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco0012.list");
//	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco006.list");
//	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco-hard1.list");
	BaseSLAM::MYNTEYEReader img_reader("/home/steve/SourceData/MYNTEYEData/aruco-hard1.list");

	cv::Mat whole_img, left_img, right_img;

	FeatureTrackServer featureTrackServer;
	featureTrackServer.setCameraParameter(stereo_camera_ptr->M1, stereo_camera_ptr->D1);

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

	int cam_id_offset = 900000;
	int feature_offset = 600000;

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
//	globalOptimizer.addParameter(camera_parameter);
	cv::Mat cam_matrix;
	cam_matrix = stereo_camera_ptr->M1;
	cam_matrix.at<double>(0, 0) = cam_matrix.at<double>(1, 1);
	std::cout << "modified cam matrix:" << cam_matrix << std::endl;
	g2o::CameraParameters *camera = new g2o::CameraParameters(cam_matrix.at<double>(0, 0),
	                                                          Eigen::Vector2d(cam_matrix.at<double>(0, 2),
	                                                                          cam_matrix.at<double>(1, 2)), 0.0
	);
	camera->setId(cam_id_offset);
	globalOptimizer.addParameter(camera);


	std::map<int, std::vector<int>> feature_frames_map;

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
	int edge_index = 1;
	Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
	for (int i = 0; i < img_reader.vec_size_; i += 3) {
		whole_img = img_reader.get_image(i);
		std::cout << " readed " << i << "-th image" << std::endl;


		left_img = img_reader.copy_left_img(whole_img);
		right_img = img_reader.copy_right_img(whole_img);

		cv::cvtColor(left_img, left_img, cv::COLOR_BGR2GRAY);

//		if(i%3==0)
		featureTrackServer.addNewFrame(left_img);
		std::cout << featureTrackServer.cur_frame_id_ << std::endl;

		Eigen::Isometry3d update_T = Eigen::Isometry3d::Identity();

		if (featureTrackServer.cur_frame_id_ > 3 && featureTrackServer.R.size().height > 0) {

			featureTrackServer.R.reshape(3, 3);
			featureTrackServer.Tt.reshape(3, 1);
			for (int x = 0; x < 3; ++x) {
				for (int y = 0; y < 3; ++y) {
					update_T.matrix()(x, y) = featureTrackServer.R.at<double>(x, y);
				}
				update_T.matrix()(x, 3) = featureTrackServer.Tt.at<double>(x);
			}
			current_pose = update_T * current_pose;

		}


		std::vector<cv::Point2f> un_pts;
		cv::undistortPoints(featureTrackServer.cur_pts_, un_pts,
		                    stereo_camera_ptr->M1, stereo_camera_ptr->D1, cv::noArray(), cam_matrix);


		g2o::VertexSE3Expmap *current_cam_vertex = new g2o::VertexSE3Expmap();
		current_cam_vertex->setId(featureTrackServer.cur_frame_id_);
//		current_cam_vertex->setEstimate(current_pose);
		current_cam_vertex->setEstimate(
				g2o::SE3Quat(
						current_pose.rotation().matrix(),
						current_pose.matrix().block(0, 3, 3, 1)
				)
		);
		if (featureTrackServer.cur_frame_id_ == 1) {
			current_cam_vertex->setFixed(true);
		}
		globalOptimizer.addVertex(current_cam_vertex);


		Eigen::Matrix3d points_info = Eigen::Matrix3d::Identity() * 50.0;


		for (int fi = 0; fi < un_pts.size(); ++fi) {
			int feature_id = featureTrackServer.ids_[fi];
			std::cout << "here feature id:" << feature_id << std::endl;
			if (feature_id > 0 && featureTrackServer.track_cnt_[fi] > 1) {
//				std::cout << "track cnt:" << featureTrackServer.track_cnt_[fi] << std::endl;
				if (featureTrackServer.track_cnt_[fi] == 2) {
//				try{
					g2o::VertexSBAPointXYZ *pointXyz = new g2o::VertexSBAPointXYZ();
					pointXyz->setId(feature_id + feature_offset);
					pointXyz->setEstimate(Eigen::Vector3d(current_pose.matrix()(0, 3),
					                                      current_pose.matrix()(1, 3),
					                                      current_pose.matrix()(2, 3)));
					globalOptimizer.addVertex(pointXyz);
//				}

				}

				g2o::EdgeProjectXYZ2UV *ob_pt = new g2o::EdgeProjectXYZ2UV();
				ob_pt->setId(edge_index);
				edge_index++;
				ob_pt->vertices()[0] = globalOptimizer.vertex(featureTrackServer.cur_frame_id_);
				ob_pt->vertices()[1] = globalOptimizer.vertex(feature_id + feature_offset);
				ob_pt->setParameterId(0, cam_id_offset);

				ob_pt->setMeasurement(Eigen::Vector2d(
						un_pts[fi].x,
						un_pts[fi].y
				));

				ob_pt->setInformation(Eigen::Matrix2d::Identity());

				globalOptimizer.addEdge(ob_pt);
			}

		}


		cv::imshow("left_src", left_img);
		cv::imshow("right_src", right_img);

//		if(i%20==0&& i <300&&i>30){
//		globalOptimizer.initializeOptimization();
//		globalOptimizer.optimize(10);
//
//		double data[10];
//		globalOptimizer.vertex(featureTrackServer.cur_frame_id_)->getEstimateData(data);
//		std::cout << "data:";
//		for(int z=0;z<10;++z){
//			std::cout << data[z]<<",";
//			out_file << data[z] << ",";
//		}
//		out_file << std::endl;
//		std::cout <<std::endl;
//		}
		cv::waitKey(10);

	}

	globalOptimizer.save("/home/steve/test_hug.g2o");

	cv::waitKey();


}