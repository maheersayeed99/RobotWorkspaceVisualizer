#include "Robot.h"

int main() {
  Robot robot;
  if (tinyxml2::XML_ERROR_PARSING_ELEMENT !=
      robot.read_urdf("../../NamingSoHard/codebase/Beta/tests/broken.urdf")) {
    return 1;
  }
  std::cout << "Success!!\n";

  return 0;
}