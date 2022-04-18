#ifndef ROBOT_IS_INCLUDED
#define ROBOT_IS_INCLUDED

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <vector>
#include <unordered_map>
#include <stdio.h>
#include <thread>
#include <future>
#include <condition_variable>
#include <mutex>
#include <functional>
#include <string>

// #include <pcl/io/pcd_io.h>  // include libraries
// #include <pcl/point_types.h>

// hashfunction for unordered_map
struct hashFunction {
  size_t operator()(const std::vector<double> &myVector) const {
    std::hash<double> hasher;
    size_t answer = 0;

    for (int i : myVector) {
      answer ^= hasher(i) + 0x9e3779b9 + (answer << 6) + (answer >> 2);
    }
    return answer;
  }
};

class Robot {
 public:
  Robot();

  class Joint {
    friend Robot;
    typedef enum { revolute, prismatic } joint_type;
    std::string frame_name_;
    std::string parent_name_;
    Eigen::Vector3d origin_xyz_;
    Eigen::Vector3d origin_rpy_;
    Eigen::Vector3d axis_;
    double joint_state_;  // default angle of the joint
    double joint_limits_[2];
  };

  /* read_urdf
  populates a vector of Joint objects (already a class attribute): joints_
  */
  bool read_urdf(std::string filepath);

  /* forward_kinematics
  summary: the function takes in a batch of configurations and computes
  corresponding end-effector positions param configs: 1st dim - an array
  (vector) of different configurations; dim1 = number of configs 2nd dim - an
  array (vector) of settings (i.e. length/angle) for each Joint (idx_0 is base
  and idx_end is end-effector); dim2 = joints.size() return: a 2d vector 1st dim
  - various configurations 2nd dim - xyz cooridnates
  */

  std::vector<std::vector<double>> forward_kinematics(
      std::vector<std::vector<double>> configs) const;

  /* get_workspace
  store everything in the hashmap: point_cloud_
  remember to check if there is an existing value for each key
  if so, stack all results in a new nested vector
  */
  void get_workspace(std::vector<std::vector<double>> configs,
                     const int nThreads);
  void save2map(std::vector<std::vector<double>> &pos,
                std::vector<std::vector<double>> &configs);

  /* save2pcd
  read all the keys from the hashmap and store in pcd file
  */
  bool save2pcd(std::string filepath);

  //  set as public for testing
 public:
  std::vector<Joint> joints_;
  std::unordered_map<std::vector<double>, std::vector<std::vector<double>>,
                     hashFunction>
      point_cloud_;

  /*=====================Helper Functions====================*/
 public:
  void print_1dVec(std::vector<double> vec);
  void print_2dVec(std::vector<std::vector<double>> vec);
  void print_map(
      std::unordered_map<std::vector<double>, std::vector<std::vector<double>>,
                         hashFunction> const &m);
};

#endif
