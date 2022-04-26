#include "Robot.h"

const double epsilon = 0.00001;

bool approximatelyEqual(double a, double b, double epsilon) {
  return std::fabs(a - b) <=
         ((std::fabs(a) < std::fabs(b) ? std::fabs(b) : std::fabs(a)) *
          epsilon);
}

int main() {
  Robot robot;
  if (0 != robot.read_urdf("../../NamingSoHard/codebase/Beta/tests/ur5.urdf")) {
    return 1;
  }
  std::vector<double> joint_angles;
  joint_angles.clear();
  joint_angles.push_back(0);
  joint_angles.push_back(0);
  joint_angles.push_back(0);
  joint_angles.push_back(0);
  joint_angles.push_back(0);
  joint_angles.push_back(0);
  auto result = robot.forward_kinematics_single(joint_angles);

  if (approximatelyEqual(result(0), 0, epsilon) &&
      approximatelyEqual(result(1), 0.19145, epsilon) &&
      approximatelyEqual(result(2), 1.00106, epsilon))
    return 0;
  else
    return 2;
}