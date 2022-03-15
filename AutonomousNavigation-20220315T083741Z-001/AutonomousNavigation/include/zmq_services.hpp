#ifndef ZMQ_SERVICES_HPP
#define ZMQ_SERVICES_HPP

#include <string>
#include "hardwareglobalinterface.h"

bool startVideoRecording(std::string filename);
bool stopVideoRecording();

bool startSpraying();
bool stopSpraying();


#endif // ZMQ_SERVICES_HPP
