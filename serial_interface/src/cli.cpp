#include <stdio.h>
#include <stdexcept>
#include "serial_interface.h"

serial_interface si_;

void data_cb(void *user_data, point p){
  //printf("phi: %f, theta: %f, r: %d\n", p.getPhi(), p.getTheta(), p.getR());
  printf("x: %f, y: %f, z: %f\n", p.getX(), p.getY(), p.getZ());
}

void new_lap_cb(void *user_data){
  printf("\n\nNew lap\n");
}

void done_cb(void *user_data){
  printf("\n\nDone\n");
  si_.stopLidar();
  exit(0);
}

int main(int argc, char const *argv[]) {
  serial_interface si = serial_interface((char*) argv[1], NULL, data_cb, new_lap_cb, done_cb);
  si_ = si;

  if (argc > 2){
    si.stopLidar();
    return 0;
  }

  si.startLidar(2);
  //for (long i = 0; i < 10000000000; i++) {
  while(true){
    try {
      si.run();
    } catch (std::runtime_error e){
      //printf("%s\n", e.what());
    }
  }
  return 0;
}
