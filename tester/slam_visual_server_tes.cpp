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

	auto visual_server = BaseSLAM::SLAMVisualServer("test");

	for(int k=0;k<1000;++k){
		cv::Affine3d fake_pose;
		fake_pose.matrix(0,3)=double(k)/500.0;
		visual_server.addNewPose(fake_pose);
		cv::waitKey(1);
	}

	cv::waitKey(0);




}