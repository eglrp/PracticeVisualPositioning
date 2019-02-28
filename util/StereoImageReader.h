//
// Created by steve on 11/15/18.
//

#ifndef BASESLAM_STEREOIMAGESERVER_H
#define BASESLAM_STEREOIMAGESERVER_H

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>


#include <boost/system/system_error.hpp>
#include <boost/filesystem.hpp>

//#include <opencv/cv.h>
#include <opencv4/opencv2/opencv.hpp>

#include <util/DataUnit.h>

namespace BaseSLAM {
	/**
	 * @brief In order to acheive better performance, load all image in memory.
	 * @warning Memory is re
	 * @tparam T
	 */
	template<typename T>
	class DataReader {
	public:
		long image_number_ = 0;

		std::string dataset_dir_;

		std::vector<T *> data_set_;


		bool thread_finished_ = false;
		std::exception *thread_except_ = nullptr;


		DataReader(std::string dir_name) {
			dataset_dir_ = dir_name;
			if (dir_name[dir_name.size() - 1] != '\\' && dir_name[dir_name.size() - 1] != '/') {
				dataset_dir_ = dataset_dir_ + '/';
				std::cout << "Dataset dir:" << dataset_dir_ << std::endl;
			}
		};

		~DataReader() {
			data_set_.clear();

		}


		/**
		 * @brief Load data.
		 * @param async using new thread or not.
		 * @return
		 */
		virtual bool load_data(bool async = true) = 0;

		/**
		 * @brief return ptr of image i;
		 * @param i
		 * @return nullptr if index not exist.
		 */
		T *get_data(long i) const {
//			std::cout << image_number_ << std::endl;

			if (i < image_number_) {
				return data_set_[i];
			} else {
				// index i out of range.
//				if (!thread_finished_) {
				// wait until i-th image is loaded or loading data thread is end.
				while (!thread_finished_ && i >= image_number_) {
					usleep(100);
				}
				if (i < image_number_) {
					return data_set_[i];
				} else {
					return nullptr;
				}

//				} else {
//					return nullptr;
//				}
			}
		}


	};


	class MYNTVIDataReader : public DataReader<StereoINSData> {
	public:
		MYNTVIDataReader(std::string dir_name) :
				DataReader<StereoINSData>(dir_name) {
			this->load_data(true);
		}


		bool load_data(bool async = true);


	};

	/**
	 * @brief Load data from disk to RAM.
	 * @param async
	 * @return
	 */
	bool MYNTVIDataReader::load_data(bool async) {

		/**
		 * @brief Just modified this function for different dataset.
		 */
		auto load_func = [&]() {
			try {
				std::vector<boost::filesystem::path> path_vec;
				std::copy(boost::filesystem::directory_iterator(dataset_dir_ + "image_0/"),
				          boost::filesystem::directory_iterator(), back_inserter(path_vec));

				std::sort(path_vec.begin(), path_vec.end());
				std::string left_file_name("");
				std::string right_file_name("");

				for (int i(0); i < path_vec.size(); ++i) {
					left_file_name = path_vec[i].c_str();
					right_file_name = left_file_name;
					int index = right_file_name.find("image_0");
					right_file_name.replace(index, 7, "image_1");

//					std::cout << "left file:" << left_file_name
//					          << "\nright file:" << right_file_name << std::endl;

					auto *data = new StereoINSData();
					cv::Mat tmp;
//					if (async == true) {

					data->loadImage(left_file_name, right_file_name, cv::IMREAD_GRAYSCALE);
//					}

					data_set_.push_back(data);
					image_number_ = data_set_.size();

//					std::cout << data->left_img_ptr_->size() << "-<>-" << data->right_img_ptr_->size() << std::endl;
//					std::cout << "size" << data_set_.size() << std::endl;


				}
				thread_finished_ = true;

			} catch (std::exception &e) {
				std::cout << e.what() << std::endl;
				*thread_except_ = e;
				thread_finished_ = true;
			}


		};
		thread_finished_ = false;


		if (async) {
			std::thread t(load_func);
			t.detach();

		} else {

			load_func();

		}
		return true;


	}


}


#endif //BASESLAM_STEREOIMAGESERVER_H
