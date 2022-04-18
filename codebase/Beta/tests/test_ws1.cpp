/*
TEST ONE:
Here we test whether get_workspace is able to properly retrieve
results (end posistions of the robot arm) from multiple threads
while storing them in correct correspondence with the inputs (configurations).
*/

#include "Robot.h"

int main() {
  Robot robot;

  /* generate fake inputs
  This particular configuration batch consists of 5 different
  configs for a 3-joint robotic arm. Thus configs is a 16x3
  vector. Thus we should expect the return value from get_workspace
  to be a 16x3 vector.
  */
  int njoints = 4;
  int nconfigs = 16;
  std::vector<std::vector<double>> configs;
  for (int i = 1; i <= nconfigs; ++i)
  {
    std::vector<double> single_config(njoints,i); // here the actual coonfigs are just i
    configs.push_back(single_config);
  }
  robot.get_workspace(configs, 4);

  /*
  Here if we print the map, we should see the map keys being
  10 times larger than the map values, e.g.

  Map Key
  =========== vector begins =========
  [ 100 100 100 ]
  ============ vector ends ==========
  Map Value
  =========== vector begins =========
  [ 10 10 10 10 ]
  ============ vector ends ==========
  */
  robot.print_map(robot.point_cloud_);

  // test whether the aforementioned is true
  for (auto const &pair : robot.point_cloud_)
  {
    if (pair.first[0] != pair.second[0][0]*10)
    // pair.first is the map key
    // pair.second is the map value
      return 1;
  }
  return 0;
}