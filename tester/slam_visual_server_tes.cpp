//
// Created by steve on 2/28/19.
//

#include <iostream>

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>


#include "VisualizeTools/SLAMVisualServer.h"
#include "VisualizeTools/SLAMVisualServer.cpp"

int main()
{

//	auto visual_server = BaseSLAM::SLAMVisualServer("test");
	BaseSLAM::SLAMVisualServer visual_server("test");

	for(int k=0;k<10000;++k){
		cv::Affine3d fake_pose;
		fake_pose.matrix(0,3)=sin(double(k)/500.0)*50.0;
		fake_pose.matrix(1,3)=cos(double(k)/500.0)*50.0;
		fake_pose.matrix(2,3) = sin(double(k)/200.0)*50.0;
		visual_server.addOdometryNewPose(fake_pose);
		cv::waitKey(100);
	}

	cv::waitKey(0);


	return 0;


}