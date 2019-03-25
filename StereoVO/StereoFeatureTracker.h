//
// Created by steve on 3/25/19.
//

#ifndef PRACTICEVISUALPOSITIONING_STEREOFEATURETRACKER_H
#define PRACTICEVISUALPOSITIONING_STEREOFEATURETRACKER_H

#include <opencv2/opencv.hpp>

#include <StereoVO/StereoConfigServer.h>

class StereoFeatureTracker {
public:




	StereoConfigServer * config_ptr_ = StereoConfigServer::getInstance();


};


#endif //PRACTICEVISUALPOSITIONING_STEREOFEATURETRACKER_H
