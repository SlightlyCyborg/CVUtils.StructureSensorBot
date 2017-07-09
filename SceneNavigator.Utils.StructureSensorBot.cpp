#include "Utils.StructureSensorBot.h"


int main(int argc, char** argv){
  SerialConfig sc = SerialConfig_from_main_args(argc, argv);
  SceneNavigator sn(sc);
  sn.run();
}
