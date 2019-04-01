//
// Created by steve on 3/29/19.
//


#include <iostream>
#include <fstream>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <util/GeoTools.h>

#include "AWF.h"

#include <VisualizeTools/SLAMVisualServer.h>

#include <opencv2/opencv.hpp>

#include <StereoVO/StereoFeatureManager.h>
#include <StereoVO/StereoConfigServer.h>


#include <VOSimulator/CameraProject.h>

int main() {

	auto logger_ptr = AWF::AlgorithmLogger::getInstance();


	auto slam_visulizer = BaseSLAM::SLAMVisualServer("gt trace");

	std::string dir_name = "/home/steve/temp/";

//	AWF::FileReader sim_frame_file(dir_name + "sim_frame_kpts.csv"),
//			sim_rframe_file(dir_name + "sim_rframe_kpts.csv"),
	AWF::FileReader sim_pos_file(dir_name + "sim_pos.csv"),
			sim_qua_file(dir_name + "sim_qua.csv"),
			kpts_file(dir_name + "kp_points.csv");

	Eigen::MatrixXd sim_pos = sim_pos_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd sim_qua = sim_qua_file.extractDoulbeMatrix(",");
	Eigen::MatrixXd kpts3 = kpts_file.extractDoulbeMatrix(",");

	std::cout << "pos size:" << sim_pos.rows() << "," << sim_pos.cols() << std::endl;
	std::cout << "qua size:" << sim_qua.rows() << "," << sim_qua.cols() << std::endl;
	std::cout << "kpts3 size:" << kpts3.rows() << "," << kpts3.cols() << std::endl;

//	cv::Mat cloud(3,kpts3.rows(),CV_32F);
	std::vector<cv::Point3d> cloud;

	for (int i = 0; i < kpts3.rows(); ++i) {
//		std::cout << kpts3.block<1, 3>(i, 0) << std::endl;
		cloud.push_back(cv::Point3f(kpts3(i, 0),
		                            kpts3(i, 1),
		                            kpts3(i, 2)));
	}
	slam_visulizer.addNewCloud("gt_points", cloud);


	CameraProject left_cameraProject(300.0, 300.0, 720, 1280);
	CameraProject right_cameraProject(300.0, 300.0, 720, 1280);

	Eigen::Quaterniond left_bc = Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY()) *
	                             Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX());

	left_cameraProject.setBody2Cam(
			left_bc, Eigen::Vector3d(0, 0, 0)
	);
	right_cameraProject.setBody2Cam(
			left_bc, Eigen::Vector3d(0.5, 0.0, 0.0)
	);

	StereoConfigServer *config_ptr_ = StereoConfigServer::getInstance();
	cv::Mat cam_mat(3, 3, CV_32F, cv::Scalar(0.0));
	cam_mat.at<float>(0, 0) = left_cameraProject.fx_;
	cam_mat.at<float>(1, 1) = left_cameraProject.fy_;
	cam_mat.at<float>(0, 2) = left_cameraProject.cx_;
	cam_mat.at<float>(1, 2) = left_cameraProject.cy_;
	cam_mat.at<float>(2, 2) = 1.0;
	cam_mat.copyTo(config_ptr_->left_cam_mat);
	cam_mat.copyTo(config_ptr_->right_cam_mat);
	cv::Mat dist_coeff(5, 1, CV_32F, cv::Scalar(0.0));
	dist_coeff.copyTo(config_ptr_->left_dist_coeff);
	dist_coeff.copyTo(config_ptr_->right_dist_coeff);

	Eigen::Matrix4d left_b2c = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d right_b2c = Eigen::Matrix4d::Identity();

	left_b2c.block<3, 3>(0, 0) = left_cameraProject.qua_bc.toRotationMatrix() * 1.0;
	left_b2c.block<3, 1>(0, 3) = left_cameraProject.t_bc * 1.0;

	right_b2c.block<3, 3>(0, 0) = right_cameraProject.qua_bc.toRotationMatrix() * 1.0;
	right_b2c.block<3, 1>(0, 3) = right_cameraProject.t_bc * 1.0;

	config_ptr_->left_bodyTocam = left_b2c * 1.0;
	config_ptr_->right_bodyTocam = right_b2c * 1.0;


	auto feature_manager_ptr_ = new StereoFeatureManager();


	for (int kk(0); kk < 100; ++kk)
		for (int i = 0; i < sim_pos.rows(); ++i) {
			logger_ptr->addTrace3dEvent(
					"src_trace",
					"gt_trace",
					sim_pos.block<1, 3>(i, 0)
			);

			slam_visulizer.addOdometryNewPose(
					sim_pos.block<1, 3>(i, 0).transpose(),
					Eigen::Quaterniond(sim_qua(i, 0),
					                   sim_qua(i, 1),
					                   sim_qua(i, 2),
					                   sim_qua(i, 3)),
					"ground truch"
			);

//		Eigen::MatrixXd pt_3d(kpts3.rows(), kpts3.cols());
			Eigen::MatrixXd pts_cam(kpts3.rows(), kpts3.cols());
			Eigen::MatrixXd r_pts_cam(kpts3.rows(), kpts3.cols());
			pts_cam.setZero();
			left_cameraProject.projectToimage(
					Eigen::Quaterniond(sim_qua(i, 0), sim_qua(i, 1), sim_qua(i, 2), sim_qua(i, 3)),
					sim_pos.block<1, 3>(i, 0).transpose(),
					kpts3,
					pts_cam
			);

			right_cameraProject.projectToimage(
					Eigen::Quaterniond(sim_qua(i, 0), sim_qua(i, 1), sim_qua(i, 2), sim_qua(i, 3)),
					sim_pos.block<1, 3>(i, 0).transpose(),
					kpts3,
					r_pts_cam
			);

			cv::Mat f_mat(left_cameraProject.height_,
			              left_cameraProject.width_,
			              CV_8UC3,
			              cv::Scalar(0, 0, 0));
			std::cout << "i:" << i <<
			          "/" << sim_pos.rows() << std::endl;


			// region Draw feature points in image.
			for (int r = 0; r < pts_cam.rows(); ++r) {

				if (pts_cam(r, 2) > 0.0) {
					cv::circle(
							f_mat,
							cv::Point2f(pts_cam(r, 0), pts_cam(r, 1)),
							5,
							cv::Scalar(0, 0, 250));

					cv::putText(f_mat,
					            std::to_string(r),
					            cv::Point2f(pts_cam(r, 0), pts_cam(r, 1)),
					            1,
					            1.0,
					            cv::Scalar(100, 100, 0));
				}

				if (r_pts_cam(r, 2) > 0.0) {
					cv::circle(
							f_mat,
							cv::Point2f(r_pts_cam(r, 0), r_pts_cam(r, 1)),
							5,
							cv::Scalar(0, 250, 0)
					);
				}

			}

			cv::imshow("test", f_mat);

			//endregion

			std::vector<cv::Point2f> feature_pts;
			std::vector<cv::Point2f> r_feature_pts;
			std::vector<int> ids;
			std::vector<int> r_ids;

			for (int r(0); r < pts_cam.rows(); ++r) {
				if (pts_cam(r, 2) > 0) {
					ids.push_back(r);
					feature_pts.push_back(cv::Point2f(pts_cam(r, 0), pts_cam(r, 1)));
					if (r_pts_cam(r, 2) > 0) {
						r_ids.push_back(r);
						r_feature_pts.push_back(cv::Point2f(r_pts_cam(r, 0), r_pts_cam(r, 1)));
						// test triangulation function
						if ((r_pts_cam.block<1, 2>(r, 0) - pts_cam.block<1, 2>(r, 0)).norm() > 5.0) {
							Eigen::Matrix3d left_R = config_ptr_->left_bodyTocam.block<3, 3>(0, 0) *
							                         Eigen::Quaterniond(sim_qua(i, 0), sim_qua(i, 1), sim_qua(i, 2),
							                                            sim_qua(i, 3)).inverse().toRotationMatrix();
							Eigen::Matrix3d right_R = config_ptr_->right_bodyTocam.block<3, 3>(0, 0) *
							                          Eigen::Quaterniond(sim_qua(i, 0), sim_qua(i, 1), sim_qua(i, 2),
							                                             sim_qua(i, 3)).inverse().toRotationMatrix();

							Eigen::Vector3d left_t = left_R * sim_pos.block<1, 3>(i, 0).transpose()  * -1.0+
							                         config_ptr_->left_bodyTocam.block<3, 1>(0, 3);
							Eigen::Vector3d right_t = right_R * sim_pos.block<1, 3>(i, 0).transpose() * -1.0 +
							                          config_ptr_->right_bodyTocam.block<3, 1>(0, 3);

							Eigen::Vector2d left_ob(pts_cam(r,0),pts_cam(r,1)),right_ob(r_pts_cam(r,0),r_pts_cam(r,1));
							Eigen::Vector3d pt3d(0,0,0);
							if(triangulatePointCeres(
									Eigen::Quaterniond(left_R),left_t,
									Eigen::Quaterniond(right_R), right_t,
									config_ptr_->right_cam_mat,left_ob,right_ob,
									pt3d
									)){
								std::cout <<"estp:" << pt3d.transpose() << std::endl;
								std::cout<<"real:" << kpts3.block<1,3>(r,0) << std::endl;

							}else{
								std::cout << "faild to calculate:"
								<< left_ob.transpose() << ":" << right_ob.transpose() << std::endl;
							}

						}


					} else {
						r_ids.push_back(-1);
						r_feature_pts.push_back(cv::Point2f(-1.0, -1.0));
					}


					// test solve pnp.

				}
			}

			assert(feature_pts.size() == r_feature_pts.size());

//			feature_manager_ptr_->addNewFrame(i,
//			                                  ids, feature_pts, r_ids, r_feature_pts);


			// tracking


			cv::waitKey(10);
//		usleep(10000);
		}


	logger_ptr->outputAllEvent(true);
	cv::waitKey();

	return 0;

}