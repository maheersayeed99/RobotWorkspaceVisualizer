/*
TEST TWO:
Here we test whether get_workspace is able to deal with circumstance
when there are multiple configurations that generate the same end position.
In other words, it should store multiple values for the same key in the hashmap.
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
    std::vector<double> single_config;
    single_config.push_back(i % 3 + 1); // here the first element could only be 1, 2, 3
    single_config.push_back(i);
    single_config.push_back(i);
    single_config.push_back(i);
    configs.push_back(single_config);
  }
  robot.get_workspace(configs, 4);

  /*
  Here if we print the map, we should see each map key have
  multiple corresponding map values, e.g.

  Map Key
  =========== vector begins =========
  [ 10 10 10 ]
  ============ vector ends ==========
  Map Value
  =========== vector begins =========
  [ 1 3 3 3 ]
  [ 1 6 6 6 ]
  [ 1 9 9 9 ]
  [ 1 12 12 12 ]
  [ 1 15 15 15 ]
  ============ vector ends ==========
  */
  robot.print_map(robot.point_cloud_);

  // test whether the aforementioned is true
  for (auto const &pair : robot.point_cloud_)
  {
    // pair.first is the map key
    // pair.second is the map value
    for (auto const &value : pair.second)
    {
      if (pair.first[0] != value[0] * 10)
        return 1;
    }
  }
  return 0;
}