#include "Robot.h"

int main() {
  Robot robot;
  if (0 != robot.read_urdf("../NamingSoHard/codebase/Beta/tests/ur5.urdf")) {
    return 1;
  }
  std::cout << "Success!!\n";

  return 0;
}