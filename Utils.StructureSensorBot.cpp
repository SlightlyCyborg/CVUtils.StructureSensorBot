//STD
#include <unistd.h>
#include <string>
#include <vector>

//3rd Party
#include <opencv2/opencv.hpp>
#include <boost/program_options.hpp>

//This lib
#include "Utils.StructureSensorBot.h"
#include "motors.h"

using namespace cv;
namespace po = boost::program_options;

void print_diagnostics(Mat &m){
  double min, max;
  std::string r;

  //Get min max vals
  minMaxIdx(m, &min, &max);

  //Get type string
  switch ( m.type() ) {
  case CV_8U:  r = "8U"; break;
  case CV_8S:  r = "8S"; break;
  case CV_16U: r = "16U"; break;
  case CV_16S: r = "16S"; break;
  case CV_32S: r = "32S"; break;
  case CV_32F: r = "32F"; break;
  case CV_64F: r = "64F"; break;
  default:     r = "User"; break;
  }

 
  printf("Mat type: %s\n",r.c_str());

  printf("Pixel Values: %f, %f", min, max);

}

void cv_points_to_vals(Mat &img, std::vector<Point> points, unsigned char* val_buf){
  std::vector<Point>::iterator it;
  int i=0;

  for(it=points.begin(); it!=points.end(); it++){
    val_buf[i] = img.at<uchar>(*it);
    i++;
  }
}

int avg_val(unsigned char* vals, int num_vals){
  short i;
  int avg = 0;

  for(i=0; i<num_vals; i++){
    avg += vals[i];
  }
  avg/=num_vals;

  return avg;
}


SerialConfig SerialConfig_from_main_args(int argc, char** argv){
  po::options_description desc("SerialConfig options");
  desc.add_options()
    ("serial_port,s", po::value<std::string>()->default_value("/dev/ttyACM0"), 
     "Serial port StructureSensorBot is on")

    ("zpos, z", po::value<int>()->default_value(90), 
     "Serial port StructureSensorBot is on")

    ("ypos, y", po::value<int>()->default_value(60), 
     "Serial port StructureSensorBot is on")

    ("baud,b", po::value<int>()->default_value(9600),
     "baud rate StructureSensorBot is communicating on");

  po::variables_map vm;
  po::store(parse_command_line(argc, argv, desc), vm);

  SerialConfig rv = {
    vm["serial_port"].as<std::string>(),
    vm["baud"].as<int>()
  };

  return rv;
}

Mover::Mover(SerialConfig sc){
  m.init_serial(sc.port_name, sc.baud);
  sleep(2);
}


void Mover::run(int argc, char** argv){

  po::options_description desc("Mover options");
  desc.add_options()
    ("serial_port,s", po::value<std::string>()->default_value("/dev/ttyACM0"), 
     "Serial port StructureSensorBot is on")

    ("z", po::value<int>()->default_value(-1), 
     "Serial port StructureSensorBot is on")

    ("y", po::value<int>()->default_value(-1), 
     "Serial port StructureSensorBot is on")

    ("baud,b", po::value<int>()->default_value(9600),
     "baud rate StructureSensorBot is communicating on");



  po::variables_map vm;
  po::store(parse_command_line(argc, argv, desc), vm);


  int z = vm["z"].as<int>();
  int y = vm["y"].as<int>();

  if(z!=-1 && y!=-1){
    m.move(z, y);
    sleep(1);
  }else if(z==-1 && y ==-1){
      FILE *fifo;
      fifo = fopen("mover_pipe", "r");
       

      while(true){
	int i = fscanf(fifo, "%d,%d\n", &z, &y);
	if(z==0 || y ==0){
	  break;
	}
	if(i == 2){
	  printf("Moving to %d, %d\n", z, y);
	  m.move(z, y);
	}
      }
      fclose(fifo);
    } 
} 


SceneNavigator::SceneNavigator(SerialConfig sc){
  m.init_serial(sc.port_name, sc.baud);
  sleep(2);
}

void cv_print(Mat m, char* txt, int x, int y, bool white){
  Scalar *c;
  if(white){
    c = new Scalar(255,0,0);
  }else{
     c = new Scalar(0,0,0);
  } 
  putText(m, txt, Point(x, y), FONT_HERSHEY_COMPLEX_SMALL, 0.6, *c, 1, CV_AA );
  delete c;
}


void SceneNavigator::run(){

  //primitive declarations
  char c;
  char txt_buf[200];

  double min, max;

  bool motor_mode = true;
  bool text_color = false;
  bool circle_blink = true;
  bool cropped = true;

  int thresh_min = -1;
  int thresh_max = -1;
  int delta = 5;
  int frame = -1;

  //Number of frames per blink
  int blink_every = 5;

  //stdlib declarations
  std::vector<Point> points;
  std::vector<Point>::iterator it;
  

  //OpenCV declarations

  Mat depth;
  Point p(400,20);
  Point cropped_ul, cropped_lr;
  const Scalar color(255,255,255);
  VideoCapture cam(CAP_OPENNI2_ASUS);
  unsigned char vals[points.size()];


  while(c = waitKey(50)){



    cam.grab();
    cam.retrieve( depth, CV_CAP_OPENNI_DEPTH_MAP );
    frame++;


    //Run diagnostics after a few frames
    if(frame == 100){
      print_diagnostics(depth);
    }

    if(cropped){
    }

    minMaxIdx(depth, &min, &max);
    depth.convertTo(depth, CV_16U, (255*256)/(max-min), -min);


    sprintf(txt_buf, "<w/s/a/d> movement, <m> motor/point mode, <c> toggle text color");
    cv_print(depth, txt_buf, 20, 20, text_color);

    sprintf(txt_buf, "<p> add to list of points, <->set thresh min (closer), <+>set thresh max");
    cv_print(depth, txt_buf, 20, 40, text_color);

    //Print point
    sprintf(txt_buf, "point x:%d y:%d; motor_mode:%d, motor_pos x:%d y:%d, thresh: max:%d, min:%d"
	    ,p.x, p.y, motor_mode, m.x_pos, m.y_pos, thresh_max, thresh_min);
    cv_print(depth, txt_buf, 20, 60, text_color);


    switch(c){
    case('p'):
      points.push_back(p);
      p = Point(400,20);
      break;
    case('m'):
      motor_mode = !motor_mode;
      break;
    case('w'):
      if(motor_mode) m.move_up(delta);
      else p.y -= delta;
      break;
    case('s'):
      if(motor_mode) m.move_down(delta);
      else p.y += delta;
      break;
    case('d'):
      if(motor_mode) m.move_right(delta);
      else p.x += delta;
      break;
    case('a'):
      if(motor_mode) m.move_left(delta);
      else p.x -= delta;
      break;
    case('+'):
      cv_points_to_vals(depth, points, vals);
      thresh_max = avg_val(vals,points.size());
      break;
    case('-'):
      cv_points_to_vals(depth, points, vals);
      thresh_min = avg_val(vals,points.size());
      break;
    case('c'):
      text_color = !text_color;
      break;
    case('\n'):
      printf("<cmd> ");
      printf("-t %d ", (thresh_max - thresh_min)/2 + thresh_min);
      printf("--xpos %d --ypos %d ", m.x_pos, m.y_pos);
      for(it=points.begin(); it != points.end(); ++it){
	printf("-x %d -y %d ", it->x, it->y);
      }
      printf("\n");
      points.clear();
      thresh_min = -1;
      thresh_max = -1;
      break;
    case('q'):
      return;
    }
  

    //Toggle blinking?
    if(frame%blink_every == 0){
      circle_blink = !circle_blink;
    }

    if(circle_blink){
      circle(depth, p, 4, color, 2);
    }

    for(it=points.begin(); it != points.end(); ++it){
      int x, y;

      circle(depth, *it, 4, color, 2);
      sprintf(txt_buf, "%d", depth.at<uchar>(*it));

      x = it->x + 5;
      y = it->y + 2;
      cv_print(depth, txt_buf, x, y, text_color);
    }

    imshow("SceneNavigator", depth);
  }
}


