#include "Robot.h"
#include <iostream>

#include <chrono>

#include <fssimplewindow.h>
#include "ysclass.h"
#include <ysglfontdata.h>

const double PI = 3.14159276;

class ApplicationMain {
 private:
  bool terminate = false;
  double FOV = PI / 6.0;  // 30 degrees
  double viewDistance = 10.0;
  YsMatrix3x3 viewRotation;
  YsVec3 viewTarget;

 public:
  ApplicationMain();
  bool MustTerminate(void) const;
  void RunOneStep(void);
  void Draw(void) const;

  decltype(std::chrono::high_resolution_clock::now()) lastT;
};

int main(int argc, char *argv[]) {
  Robot robot;

  std::vector<double> temp{0, 0, 0, 0, 0};
  std::vector<std::vector<double>> configs{temp, temp, temp, temp,
                                           temp, temp, temp, temp};
  robot.get_workspace(configs, 8);
  robot.print_map(robot.point_cloud_);
  return 0;

  std::cout << "Hello world \n";
}