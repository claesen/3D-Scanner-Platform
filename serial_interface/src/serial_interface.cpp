// C library headers
#include <stdio.h>
#include <string.h>

#include <iostream>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <stdexcept>
#include <string>

#include "serial_interface.h"

serial_interface::serial_interface(char *interface, void *user_data, std::function<void(void*, point)> data_cb, std::function<void(void*)> new_lap_cb, std::function<void(void*)> done_cb){
  this->user_data = user_data;

  if (data_cb == NULL){
    throw std::runtime_error("Error invalid data callback");
  }
  this->data_cb = data_cb;

  if (new_lap_cb == NULL){
    throw std::runtime_error("Error invalid new lap callback");
  }
  this->new_lap_cb = new_lap_cb;

  if (done_cb == NULL){
    throw std::runtime_error("Error invalid done callback");
  }
  this->done_cb = done_cb;

  // Credit for serial interface setup goes to https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
  this->serial_port = open(interface, O_RDWR);

  // Check for errors
  if (this->serial_port < 0) {
    throw std::runtime_error("Error " + std::to_string(errno) + " from open: " + std::string(strerror(errno)));
  }

  memset(&this->tty, 0, sizeof this->tty);

  // Read in existing settings, and handle any error
  if(tcgetattr(this->serial_port, &this->tty) != 0) {
    throw std::runtime_error("Error " + std::to_string(errno) + " from tcgetattr: " + std::string(strerror(errno)));
  }

  this->tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  this->tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  this->tty.c_cflag &= ~CSIZE; // Clear size bits
  this->tty.c_cflag |= CS8; // 8 bits per byte (most common)
  this->tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  this->tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  this->tty.c_lflag &= ~ICANON;
  this->tty.c_lflag &= ~ECHO; // Disable echo
  this->tty.c_lflag &= ~ECHOE; // Disable erasure
  this->tty.c_lflag &= ~ECHONL; // Disable new-line echo
  this->tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  this->tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  this->tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  this->tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  this->tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  this->tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  this->tty.c_cc[VMIN] = 0;


  // Set in/out baud rate to be 115200
  cfsetispeed(&this->tty, B115200);
  cfsetospeed(&this->tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(this->serial_port, TCSANOW, &this->tty) != 0) {
      throw std::runtime_error("Error " + std::to_string(errno) + " from tcsetattr: " + std::string(strerror(errno)));
  }


  // TODO: Better wait
  for(int i = 0; i < 1000000000; i++) { }
}

serial_interface::~serial_interface(){
  close(this->serial_port);
}

int serial_interface::startLidar(){
  this->startLidar(1);
}

int serial_interface::startLidar(int n_laps){
  std::string msg = "r" + std::to_string(n_laps) + "\r";
  int n = write(this->serial_port, msg.c_str(), sizeof(char)*msg.length());
  if (n == -1) {
      throw std::runtime_error("Error " + std::to_string(errno) + " from write: " + std::string(strerror(errno)));
  }
  // TODO: Better wait
  for(int i = 0; i < 1000; i++) { }
}

int serial_interface::stopLidar(){
  unsigned char msg[] = { 's', '\r' };
  int n = write(this->serial_port, msg, sizeof(msg));
  if (n == -1) {
      throw std::runtime_error("Error " + std::to_string(errno) + " from write: " + std::string(strerror(errno)));
  }
  // TODO: Better wait
  for(int i = 0; i < 1000; i++) { }
}

int serial_interface::restartLidar(){
  this->startLidar();
}

int serial_interface::setSpeed(int speed){
  throw std::runtime_error("setSpeed not implemeted");
}

int serial_interface::setResolution(int resolution){
  throw std::runtime_error("setResolution not implemeted");
}


void serial_interface::run(){
  int len;

  char buf[1024];
  memset(buf, 0, 1024);

  len = read(this->serial_port, &buf, sizeof buf);
  if (len == -1) {
      throw std::runtime_error("Error " + std::to_string(errno) + " from read: " + std::string(strerror(errno)));
  }
    std::string buf_str(buf);
    if(buf_str.find('nack') != std::string::npos || buf_str.find('read failed') != std::string::npos){
        std::cout << buf_str << std::endl;
        return;
    }


  this->read_buf.append(buf, len);


  while (this->read_buf.length()){
    if (this->read_buf[0] != '\n') {
      this->read_buf.erase(0, 1);
    } else {
      break;
    }
  }

  int i;
  for (i = 1; i < this->read_buf.length(); i++) {
    if (this->read_buf[i] == '\n') {
      break;
    }
  }

  //printf("i:%d, len:%ld%s", i, this->read_buf.length(), this->read_buf.c_str());

  // We have found a complete message
  if (i < this->read_buf.length() || this->read_buf[i] == '\n') {
    std::string tmp = this->read_buf.substr(1, i-1);

    // Weird magic, this is needed for the if statement to work..
    //tmp.c_str();
    //printf("%s", i, this->read_buf.length(), this->read_buf.c_str());

    if (tmp[0] == 'n'){
      this->handle_new_lap();
    } else if (tmp[0] == 'd'){
      this->handle_done();
    } else{
      this->handle_point(tmp);
    }

    this->read_buf.erase(0, i-1);
  } else {
    throw std::runtime_error("No new message");
  }

  return;
}

void serial_interface::handle_point(std::string s){
  float phi;
  float theta;
  int r;

  if(s.length() < 12){
    throw std::runtime_error("bla\n");
  }

  size_t idx[2];

  if(s.length() < 2){
      throw std::runtime_error("bla");
  }
  phi   = stof(s.substr(0,                   std::string::npos), &idx[0]);

  if(s.length()-idx[0] < 3){
      throw std::runtime_error("bla");
  }
  theta = stof(s.substr(1 + idx[0],          std::string::npos), &idx[1]);

  if(s.length()-idx[0]-idx[1] < 4){
      throw std::runtime_error("bla");
  }
  r     = stoi(s.substr(2 + idx[0] + idx[1], std::string::npos), NULL);

  this->data_cb(this->user_data, point(phi, theta, r));
}

void serial_interface::handle_new_lap(){
  this->new_lap_cb(this->user_data);
}

void serial_interface::handle_done(){
  this->done_cb(this->user_data);
}
