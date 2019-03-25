//
// Created by steve on 3/25/19.
//

#ifndef PRACTICEVISUALPOSITIONING_CONFIGERSERVER_H
#define PRACTICEVISUALPOSITIONING_CONFIGERSERVER_H

#include <thread>
#include <mutex>

class ConfigerServer {
public:
	static ConfigerServer* getInstance(){

		static std::once_flag oc;

		if(instance_ == nullptr){
			std::call_once(oc,[]{
				if(instance_==nullptr){
					instance_ = new ConfigerServer();
				}
			});
		}
		return instance_;
	}

protected:

	ConfigerServer(){}

	~ConfigerServer(){}

	ConfigerServer(const ConfigerServer &){}

	ConfigerServer &operator=(const ConfigerServer&){}

	static ConfigerServer *instance_;


};


#endif //PRACTICEVISUALPOSITIONING_CONFIGERSERVER_H
