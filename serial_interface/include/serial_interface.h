#ifndef SERIAL_INTERFACE
#define SERIAL_INTERFACE

#include <string>
#include <functional>
#include <cmath>
#include <termios.h> // Contains POSIX terminal control definitions

class point {
    float phi;
    float theta;
    int r;
  public:
    point(float phi, float theta, int r){
      this->phi = phi;
      this->theta = -theta;
      this->r = r;
    }
    ~point(){

    }
    float getPhi(){
      return M_PI - this->phi;
    }
    float getTheta(){
      return this->theta;
    }
    int getR(){
      return this->r;
    }
    float getX(){
      return this->r*sin(M_PI - phi)*cos(theta);
    }
    float getY(){
      return this->r*sin(M_PI - phi)*sin(theta);
    }
    float getZ(){
      return this->r*cos(M_PI - phi);
    }
};

class serial_interface {
    int serial_port;
    struct termios tty;
    std::string read_buf;
    void handle_point(std::string);
    void handle_new_lap();
    void handle_done();
    void *user_data;
    std::function<void(void*, point)> data_cb;
    std::function<void(void*)> new_lap_cb;
    std::function<void(void*)> done_cb;
  public:
    serial_interface(char*, void*, std::function<void(void*, point)>, std::function<void(void*)>, std::function<void(void*)>);
    serial_interface(){

    }
    ~serial_interface();
    int startLidar();
    int startLidar(int);
    int stopLidar();
    int restartLidar();
    int setSpeed(int);
    int setResolution(int);
    void run();
};

#endif /* end of include guard: SERIAL_INTERFACE */
