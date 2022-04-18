#include "Robot.h"

Robot::Robot() { ; }

void Robot::print_1dVec(std::vector<double> vec) {
  std::cout << " =========== vector.begin() ========= " << std::endl;
  std::cout << "[ ";
  for (int j = 0; j < vec.size(); ++j) {
    std::cout << vec[j] << " ";
  }
  std::cout << "] ";
  std::cout << std::endl;
  std::cout << " ============ vector.end() ========== " << std::endl;
}

void Robot::print_2dVec(std::vector<std::vector<double>> vec) {
  std::cout << " =========== vector.begin() ========= " << std::endl;
  for (int i = 0; i < vec.size(); ++i) {
    std::cout << "[ ";
    for (int j = 0; j < vec[i].size(); ++j) {
      std::cout << vec[i][j] << " ";
    }
    std::cout << "] ";
    std::cout << std::endl;
  }
  std::cout << " ============ vector.end() ========== " << std::endl;
}

void Robot::print_map(
    std::unordered_map<std::vector<double>, std::vector<std::vector<double>>,
                       hashFunction> const &m) {
  for (auto const &pair : m) {
    std::cout << "\nMap Key" << std::endl;
    print_1dVec(pair.first);

    std::cout << "Map Value" << std::endl;
    print_2dVec(pair.second);
    std::cout << std::endl;
    // std::cout << ": Value" << std::endl;
    // print_2dVec(pair.second);
    // std::cout << "}\n";
  }
}

// bool Robot::save2pcd(std::string filepath)
// {

//     pcl::PointCloud<pcl::PointXYZ> cloud; // make point cloud object

//     cloud.points.resize(joints_.size()); // resize cloud

//     int i = 0;
//     for (auto &point : cloud)
//     {
//         point.x = joints_[i].x(); // fill cloud
//         point.y = joints_[i].y();
//         point.z = joints_[i].z();

//         i++;
//     }

//     if (pcl::io::savePCDFileASCII(filepath, cloud))
//     { // save to file
//         return 1;
//     }
//     else
//     {
//         return 0;
//     }
// }
