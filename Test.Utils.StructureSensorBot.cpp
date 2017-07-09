#include <opencv2/opencv.hpp>
#include "motors.h"
#include "Utils.StructureSensorBot.h"
#include <stdio.h>

int main(int argc, char** argv){
  printf("STARTING: Testing SerialConfig from_main_args();\n");

  SerialConfig sc = SerialConfig_from_main_args(argc, argv);

  if(sc.port_name == "/dev/ttyACM0"){
    printf("%30s", "default port args PASSED\n");
  }else{
    printf("%30s", "default port args FAILED\n");
  }

  if(sc.baud == 9600){
    printf("%30s", "default baud args PASSED\n");
  }else{
    printf("%30s", "default baud args FAILED\n");
  }

  printf("--ENDING: Testing SerialConfig from_main_args();\n");


  SceneNavigator sn(sc);
  sn.run();
}


