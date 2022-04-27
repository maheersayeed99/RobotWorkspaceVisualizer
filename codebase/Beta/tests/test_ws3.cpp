/*
TEST THREE:
Here we test whether get_workspace is able to properly divde work amongst
a prescribed number of threads. Particularly if the number of tasks is 
fewer than number of threads or if number of tasks is not an exact multiple
of the number of threads.
*/

#include "Robot.h"

int main()
{
  Robot robot;

  /* generate fake inputs
  This particular configuration batch consists of 5 different
  configs for a 3-joint robotic arm. Thus configs is a 7x3
  vector. Thus we should expect the return value from get_workspace
  to be a 16x3 vector.
  */
  int njoints = 4;
  int nconfigs = 7;
  std::vector<std::vector<double>> configs;
  for (int i = 1; i <= nconfigs; ++i)
  {
    std::vector<double> single_config(njoints, i); // here the actual coonfigs are just i
    configs.push_back(single_config);
  }
  robot.get_workspace(configs, 8, true); // more threads requested than needed

  /*
  Here since number of tasks is fewer than number of threads,
  we should use fewer threads than requested. In other words,
  even though we specified 8 threads, we will only end up using
  7 threads, each processing one configuration.
  If we print the map, we should see complete results.
  */
  robot.print_map(robot.point_cloud_);

  // test whether the aforementioned is true
  if (robot.point_cloud_.size() != nconfigs)
    return 1;
  return 0;
}

