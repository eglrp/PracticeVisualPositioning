//
// Created by steve on 3/25/19.
//

#include "StereoFeatureManager.h"


bool StereoFeatureManager::addNewFrame(int frame_id, std::vector<int> feature_id, std::vector<cv::Point2f> feature_pts,
                                       std::vector<int> r_feature_id, std::vector<cv::Point2f> r_feature_pts) {
	 // add frame to frame map

	 frame_map_.insert(
	 		std::make_pair(frame_id,FramePreId(frame_id))
	 		);
	 frame_map_[frame_id].

	 // check if it's needed to add a new key frame.


	 // add new key frame.





}