//
// Created by steve on 3/25/19.
//

#ifndef PRACTICEVISUALPOSITIONING_STEREOODOMETRYSERVER_H
#define PRACTICEVISUALPOSITIONING_STEREOODOMETRYSERVER_H

#include <StereoVO/StereoFeatureTracker.h>
//#include <StereoVO/StereoFeatureTracker.cpp>
#include <StereoVO/StereoFeatureManager.h>
//#include <StereoVO/StereoFeatureManager.cpp>
#include <StereoVO/Optimizer.h>
//#include <StereoVO/Optimizer.cpp>
#include <VisualizeTools/SLAMVisualServer.h>
//#include <VisualizeTools/SLAMVisualServer.cpp>

class StereoOdometryServer {
public:


	StereoOdometryServer(){
		config_ptr_ = StereoConfigServer::getInstance();

		trace_file.open("/home/steve/temp/pose.csv", std::ios::out);
	}

	bool addNewFrame(cv::Mat &left_img, cv::Mat &right_img);

	bool addNewFrame(cv::Mat &left_img){
		return addNewFrame(left_img,left_img);
	}


	StereoFeatureTracker * tracker_ptr_ = new StereoFeatureTracker();
	StereoFeatureManager * feature_manager_ptr_ = new StereoFeatureManager();
	Optimizer * optimizer_ptr_ = new Optimizer();

	StereoConfigServer * config_ptr_ ;//= StereoConfigServer::getInstance();

	BaseSLAM::SLAMVisualServer * visualizer_ptr_ = new BaseSLAM::SLAMVisualServer("VO WIN");


	std::ofstream trace_file;




};


#endif //PRACTICEVISUALPOSITIONING_STEREOODOMETRYSERVER_H
