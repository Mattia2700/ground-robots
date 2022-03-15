#include <iostream>
#include <getopt.h>

#include "hardwareglobalinterface.h"
#include "hardwareparameters.h"
#include "lidartracking.h"

bool nuc_id_specified = false;
int nucId;

void PrintHelp()
{
  std::cout <<
    "--nucID <NID>: NUC ID" << std::endl <<
    "--help: show help" << std::endl <<
    "example:" << std::endl <<
    "./lidar_tracking --nucID 3" << std::endl;
  exit(1);
}

void ProcessArgs(int argc, char** argv)
{
  const char* const short_opts = "n:h";
  const option long_opts[] = {
    {"nucID", required_argument, nullptr, 'n'},
    {"help", no_argument, nullptr, 'h'},
    {nullptr, no_argument, nullptr, 0}
  };

  while (true)
  {
    const auto opt = getopt_long(argc, argv, short_opts, long_opts, nullptr);
    if (-1 == opt)
      break;
    switch (opt)
    {
      case 'n':
        nucId = std::atoi(optarg);
        std::cout << " NUC ID: " << nucId << std::endl;
        nuc_id_specified = true;
        break;            
      case 'h': // -h or --help
      case '?': // Unrecognized option
      default:
        PrintHelp();
        break;
    }
  }
}



int main(int argc, char ** argv) 
{
  ProcessArgs(argc,  argv);
  std::cout << "TRACKING TOOL!" << std::endl;

  if (!nuc_id_specified)
  {
    PrintHelp();
    exit(1);
  }

  HardwareParameters hpRobot(nucId);
  HardwareGlobalInterface::initialize(&hpRobot);

  std::unique_ptr<LidarTracking> tracker = std::make_unique<LidarTracking>();
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  return 0;
}


