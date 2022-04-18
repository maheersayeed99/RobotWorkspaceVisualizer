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
#include <fstream>

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
  // Do not use in final version
 public:
  void print_1dVec(std::vector<double> vec);
  void print_2dVec(std::vector<std::vector<double>> vec);
  void print_map(
      std::unordered_map<std::vector<double>, std::vector<std::vector<double>>,
                         hashFunction> const &m);
  int numP;
  float rgb = 4.808e+06;
  std::vector<float> xvec, yvec, zvec;
  std::vector<float> vtx, col;
  void makePCD(int numPoints) {
    numP = numPoints;
    float theta, phi, rho;
    float x, y, z;
    for (int i = 0; i < numPoints; ++i) {
      if (i < numPoints / 3) {
        rho = 1;

        col.push_back(1);
        col.push_back(1);
        col.push_back(0);
        col.push_back(1);

      } else if (i < 2 * numPoints / 3) {
        rho = 2;

        col.push_back(1);
        col.push_back(0);
        col.push_back(1);
        col.push_back(1);
      } else {
        rho = 3;

        col.push_back(0);
        col.push_back(1);
        col.push_back(1);
        col.push_back(1);
      }

      x = ((rand() % 200) / 100.0) - 1;
      y = ((rand() % 200) / 100.0) - 1;
      z = ((rand() % 200) / 100.0) - 1;

      x *= rho;
      y *= rho;
      z *= rho;

      vtx.push_back(x);
      vtx.push_back(y);
      vtx.push_back(z);
    }
  };
  void savePCD(std::string fileName) {
    std::ofstream PCD;

    PCD.open(fileName, std::fstream::out);

    if (PCD.fail()) {
      std::cout << "could not open" << std::endl;
    }

    else {
      PCD << "FIELDS x y z rgb" << std::endl;
      PCD << "SIZE 4 4 4 4" << std::endl;
      PCD << "TYPE F F F F" << std::endl;
      PCD << "COUNT 1 1 1 1" << std::endl;
      PCD << "WIDTH " << numP << std::endl;
      PCD << "HEIGHT 1" << std::endl;
      PCD << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
      PCD << "POINTS " << numP << std::endl;
      PCD << "DATA ascii" << std::endl;

      for (int i = 0; i < xvec.size(); ++i) {
        PCD << xvec[i] << " " << yvec[i] << " " << zvec[i] << " " << rgb
            << std::endl;
      }

      PCD.close();

      std::cout << "File saved as newTest.pcd" << std::endl;
    }
  }
};

#endif
