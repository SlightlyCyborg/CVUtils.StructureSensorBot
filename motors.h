#ifndef __MOTORS_H__
#define __MOTORS_H__

#include "boost/asio/serial_port.hpp"
#include <boost/asio.hpp> 
#include <string>



using namespace boost;
using std::string;


class Motors{

  asio::io_service io;
  asio::serial_port * port;
  unsigned char c[2];

public:
  unsigned char x_pos, y_pos, x_motor_cmd, y_motor_cmd;

  ~Motors(){
    port->close();
    delete port;
  }
  Motors(){
    x_pos = 90;
    y_pos = 40;

    y_motor_cmd = 0;
    x_motor_cmd = 1;

   port = new asio::serial_port(io);

  }

  void move_up(int amt){
    y_pos += amt;
    char commands[2];
    commands[0]=0; 
    commands[1]=y_pos;
    port->write_some(boost::asio::buffer(commands, 2));
  }

  void move_down(int amt){
    y_pos -= amt;
    char commands[2];
    commands[0]=0; 
    commands[1]=y_pos;
    port->write_some(boost::asio::buffer(commands, 2));
  }

  void move_left(int amt){
    x_pos -= amt;
    char commands[2];
    commands[0]=1; 
    commands[1]=x_pos;
    port->write_some(boost::asio::buffer(commands, 2));
  }

  void move_right(int amt){
    x_pos += amt;
    char commands[2];
    commands[0]=1; 
    commands[1]=x_pos;
    port->write_some(boost::asio::buffer(commands, 2));

  }

  bool init_serial(string port_name, int baud){
    //intialize serial
   
    port->open(port_name);
    port->set_option(asio::serial_port_base::baud_rate(baud));
    // port, baudrate, timeout in milliseconds

    return port->is_open();

  }

  void return_home(){
    char commands[2];
    x_pos = 90;
    y_pos = 60;
   

    commands[0]=1; 
    commands[1]=x_pos;
    port->write_some(boost::asio::buffer(commands, 2));
    commands[0]=0; 
    commands[1]=y_pos;
    port->write_some(boost::asio::buffer(commands, 2));


  }

  void init_motors(int x, int y){

    char commands[2];
    x_pos = x;
    y_pos = y;
   

    commands[0]=1; 
    commands[1]=x_pos;
    port->write_some(boost::asio::buffer(commands, 2));
    commands[0]=0; 
    commands[1]=y_pos;
    port->write_some(boost::asio::buffer(commands, 2));
  }
};

#endif
