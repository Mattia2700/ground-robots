#include "zmq_services.hpp"
#include "RequesterSimple.hpp"

#include "json.hpp"

using namespace nlohmann;

bool startVideoRecording(std::string filename) {
  HardwareParameters params = HardwareGlobalInterface::getInstance().getParams();
  ZMQCommon::RequesterSimple req(params.videoServer);
  try {
    json j_req;
    j_req["cmd"] = std::string("record");
    j_req["name"] = filename;

    std::string response;
    ZMQCommon::RequesterSimple::status_t req_status;
    req.request(j_req.dump(), response, req_status);
    std::cout << j_req.dump() << std::endl;
    if(req_status==ZMQCommon::RequesterSimple::status_t::STATUS_OK){
      nlohmann::json j_resp = nlohmann::json::parse(response);
      std::cout << j_resp.dump()<<std::endl;
      if(j_resp.at("ack") == "true"){
      }else{
      }
    }else{

    }
  }catch(std::exception &e){
    std::cerr<<e.what()<<std::endl;
  }
}

bool stopVideoRecording() {
  HardwareParameters params = HardwareGlobalInterface::getInstance().getParams();
  ZMQCommon::RequesterSimple req(params.videoServer);
  try {
    json j_req;
    j_req["cmd"] = std::string("stop");

    std::string response;
    ZMQCommon::RequesterSimple::status_t req_status;
    req.request(j_req.dump(), response, req_status);
    std::cout << j_req.dump() << std::endl;
    if(req_status==ZMQCommon::RequesterSimple::status_t::STATUS_OK){
      nlohmann::json j_resp = nlohmann::json::parse(response);
      std::cout << j_resp.dump()<<std::endl;
      if(j_resp.at("ack") == "true"){
      }else{
      }
    }else{

    }
  }catch(std::exception &e){
    std::cerr<<e.what()<<std::endl;
  }
}

bool startSpraying() {
  return true;
  HardwareParameters params = HardwareGlobalInterface::getInstance().getParams();
  ZMQCommon::RequesterSimple req(params.sprayServer);
  try {
    json j_req;
    j_req["cmd"] = std::string("spray");

    std::string response;
    ZMQCommon::RequesterSimple::status_t req_status;
    req.request(j_req.dump(), response, req_status);
    std::cout << j_req.dump() << std::endl;
    if(req_status==ZMQCommon::RequesterSimple::status_t::STATUS_OK){
      nlohmann::json j_resp = nlohmann::json::parse(response);
      std::cout << j_resp.dump()<<std::endl;
      if(j_resp.at("ack") == "true"){
      }else{
      }
    }else{

    }
  }catch(std::exception &e){
    std::cerr<<e.what()<<std::endl;
  }
}

bool stopSpraying() {
  return true;
  HardwareParameters params = HardwareGlobalInterface::getInstance().getParams();
  ZMQCommon::RequesterSimple req(params.sprayServer);
  try {
    json j_req;
    j_req["cmd"] = std::string("stop");

    std::string response;
    ZMQCommon::RequesterSimple::status_t req_status;
    req.request(j_req.dump(), response, req_status);
    std::cout << j_req.dump() << std::endl;
    if(req_status==ZMQCommon::RequesterSimple::status_t::STATUS_OK){
      nlohmann::json j_resp = nlohmann::json::parse(response);
      std::cout << j_resp.dump()<<std::endl;
      if(j_resp.at("ack") == "true"){
      }else{
      }
    }else{

    }
  }catch(std::exception &e){
    std::cerr<<e.what()<<std::endl;
  }
}
