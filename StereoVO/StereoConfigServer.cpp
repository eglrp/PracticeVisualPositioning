//
// Created by steve on 3/26/19.
//

#include <StereoVO/StereoConfigServer.h>

StereoConfigServer *StereoConfigServer::instance_ = nullptr;


StereoConfigServer *StereoConfigServer::getInstance()  {

	static std::once_flag oc;

	if (instance_ == nullptr) {
		std::call_once(oc, [] {
			if (instance_ == nullptr) {
				instance_ = new StereoConfigServer();
			}
		});
	}
	return instance_;
}