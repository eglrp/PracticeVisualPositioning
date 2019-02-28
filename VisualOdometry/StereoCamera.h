//
// Created by steve on 11/15/18.
//

#ifndef BASESLAM_CAMERACONFIGSERVER_H
#define BASESLAM_CAMERACONFIGSERVER_H


#include <opencv2/opencv.hpp>

namespace BaseSLAM {
	class StereoCamera {
	public:

		cv::Mat M1, M2, D1, D2, R, T;

		/**
		 * @brief Load file from camera.
		 * @param file_name
		 */
		StereoCamera(const std::string &file_name) {
			cv::FileStorage fs(file_name, cv::FileStorage::READ);
			if (fs.isOpened()) {
//				M1 = (cv::Mat)(fs["M1"]);
//				M2 = (cv::Mat)(fs["M2"]);
//				D1 = (cv::Mat)(fs["D1"]);
//				D2 = (cv::Mat)(fs[""])
				fs["M1"] >> M1;
				fs["M2"] >> M2;
				fs["D1"] >> D1;
				fs["D2"] >> D2;
				fs["R"] >> R;
				fs["T"] >> T;
			}


		}


		void print(const std::string &str) {
			std::cout << str << "\n"
			          << M1 << "\n" << D1 << "\n"
			          << M2 << "\n" << D2 << "\n"
			          << R << "\n" << T << "\n"
			          << std::endl;
		}


	};

}


#endif //BASESLAM_CAMERACONFIGSERVER_H
