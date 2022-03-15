#ifndef HARDWAREPARAMETERS_H
#define HARDWAREPARAMETERS_H

#include <string>

struct HardwareParameters{
public:

  std::string hardwareServer = "tcp://192.168.3.2:5601";
  std::string hardwarePublisher = "tcp://192.168.3.2:6000";
  std::string trolleyPublisher = "tcp://192.168.3.2:6001";
  std::string localizationServer = "tcp://192.168.3.5:5600";
  std::string localizationPublisher = "tcp://192.168.3.5:5563";
  std::string frontLidarPublisher = "tcp://192.168.3.5:7500";
  std::string rearLidarPublisher = "tcp://192.168.3.5:7501";
  std::string realSensePublisher = "tcp://192.168.3.5:7502";
  std::string kartoPosePublisher = "tcp://192.168.3.99:7001";
  //std::string realSensePosePublisher = "tcp://192.168.3.5:9112";
  std::string realSensePosePublisher = "tcp://192.168.3.5:9207";
  std::string realSenseTagsPublisher = "tcp://192.168.3.5:9114";
  std::string trackingPublisher = "tcp://192.168.3.5:7901";
  std::string predictionPublisher = "tcp://192.168.3.5:9210";
  //std::string peopleNeighPublisher = "tcp://192.168.3.5:9110";
  std::string autnavServer = "tcp://192.168.3.5:1991";
  std::string autnavPublisher = "tcp://192.168.3.5:1992";
  std::string videoServer = "tcp://192.168.3.5:9115";
  std::string sprayServer = "tcp://192.168.3.2:5300";

  double vehicleWidth = 0.5;
  double vehicleLength = 0.5;
  double cameraOffsetX = 0.25;
  double cameraOffsetY = 0.0;

  HardwareParameters(){
    hardwareServer = "tcp://192.168.3.2:5601";
    hardwarePublisher = "tcp://192.168.3.2:6000";
    trolleyPublisher = "tcp://192.168.3.2:6001";
    localizationServer = "tcp://192.168.3.5:5600";
    localizationPublisher = "tcp://192.168.3.5:5563";
    frontLidarPublisher = "tcp://192.168.3.5:7500";
    rearLidarPublisher = "tcp://192.168.3.5:7501";
    realSensePublisher = "tcp://192.168.3.5:7502";
    kartoPosePublisher = "tcp://192.168.3.99:7001";
//    realSensePosePublisher = "tcp://192.168.3.5:9112";
    realSensePosePublisher = "tcp://192.168.3.5:9207";
    realSenseTagsPublisher = "tcp://192.168.3.5:9114";
    trackingPublisher = "tcp://192.168.3.5:7901";
    predictionPublisher = "tcp://192.168.3.5:9210";
    //peopleNeighPublisher = "tcp://192.168.3.5:9110";
    autnavServer = "tcp://192.168.3.5:1991";
    autnavPublisher = "tcp://192.168.3.5:1992";
    videoServer = "tcp://192.168.3.5:9115";
    sprayServer = "tcp://192.168.3.2:5300";
  }

  HardwareParameters(std::string val){
    if(val.compare("localhost")==0){
      hardwareServer = "tcp://localhost:5601";
      hardwarePublisher = "tcp://localhost:6000";
      trolleyPublisher = "tcp://localhost:6001";
      localizationServer = "tcp://localhost:5600";
      localizationPublisher = "tcp://localhost:5563";
      frontLidarPublisher = "tcp://localhost:7500";
      rearLidarPublisher = "tcp://localhost:7501";
      realSensePublisher = "tcp://localhost:7502";
      kartoPosePublisher = "tcp://localhost:7001";
//      realSensePosePublisher = "tcp://localhost:9112";
      realSensePosePublisher = "tcp://localhost:9207";
      realSenseTagsPublisher = "tcp://localhost:9114";
      trackingPublisher = "tcp://localhost:7901";
      predictionPublisher = "tcp://localhost:9210";
      //peopleNeighPublisher = "tcp://localhost:9110";  
      autnavServer = "tcp://localhost:1991";
      autnavPublisher = "tcp://localhost:1992";
      videoServer = "tcp://localhost:9115";
      sprayServer = "tcp://localhost:5300";
    }
  }

  HardwareParameters(int i){
    int bbIP = 100 + i*10 + 2;
    int nucIP = 100 + i*10 + 5;
    hardwareServer = "tcp://10.196.80." + std::to_string(bbIP) + ":5601";
    hardwarePublisher = "tcp://10.196.80." + std::to_string(bbIP) + ":6000";
    trolleyPublisher = "tcp://10.196.80." + std::to_string(bbIP) + ":6001";
    localizationServer = "tcp://10.196.80." + std::to_string(nucIP) + ":5600";
    localizationPublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":5563";
    frontLidarPublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":7500";
    rearLidarPublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":7501";
    realSensePublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":7502";
    kartoPosePublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":7001";
//    realSensePosePublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":9112";
    realSensePosePublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":9207";
    realSenseTagsPublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":9114";
    trackingPublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":7901";
    predictionPublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":9210";
    //peopleNeighPublisher = "tcp://10.196.80." + std::to_string(nucIP) + ":9110";
    autnavServer = "tcp://192.168.1." + std::to_string(i) + ":1991";
    autnavPublisher = "tcp://*:1992";
    videoServer = "tcp://10.196.80." + std::to_string(nucIP) + ":9115";
    sprayServer = "tcp://10.196.80." + std::to_string(bbIP) + ":5300";
  }
};

#endif // HARDWAREPARAMETERS_H
