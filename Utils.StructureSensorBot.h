#ifndef __UTILS_STRUCTURE_SENSOR_BOT_H__
#define __UTILS_STRUCTURE_SENSOR_BOT_H__

#include <string>
#include "motors.h"

struct SerialConfig{
  std::string port_name;
  int baud;
};


SerialConfig SerialConfig_from_main_args(int argc, char** argv);

class SceneNavigator{
  Motors m;

 public:
  SceneNavigator(SerialConfig);
  void run();
};

#endif
