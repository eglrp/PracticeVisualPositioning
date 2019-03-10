//
// Created by steve on 3/10/19.
//

#ifndef PRACTICEVISUALPOSITIONING_FEATURE_H
#define PRACTICEVISUALPOSITIONING_FEATURE_H

#include <vector>
#include <eigen3/Eigen/Geometry>

namespace BaseSLAM {
	class FeaturePoint {
	public:
		const int feature_id_;
		int start_frame_;
		int end_frame_;
		std::vector<int> countained_frame_ids_;

		int used_num_;
		bool is_outlier_;
		bool is_margin_;
		double estimated_depth_;

		Eigen::Vector3d gt_point_;

		FeaturePoint(int feature_id, int start_frame)
				: feature_id_(feature_id), start_frame_(start_frame),
				  used_num_(0), estimated_depth_(-1.0) {
			countained_frame_ids_.push_back(start_frame);
		}


		/**
		 * @brief return the end frame that detected this feature.
		 * @return
		 */
		int endFrame() {
			return countained_frame_ids_[countained_frame_ids_.size() - 1];
		}
	};


}


#endif //PRACTICEVISUALPOSITIONING_FEATURE_H
