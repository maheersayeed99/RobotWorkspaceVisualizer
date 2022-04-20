#include "Robot.h"

Robot::Robot() { ; }
void Robot::savePCD(std::string fileName) {
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

    for (auto const &pair : point_cloud_) {
      PCD << (float)pair.first[0] << " " << (float)pair.first[1] << " "
          << (float)pair.first[2] << " " << rgb << std::endl;
    }

    /*
    for (int i = 0; i < xvec.size(); ++i) {
      PCD << point_cloud_[i] << " " << yvec[i] << " " << zvec[i] << " " << rgb
          << std::endl;
    }
    */
    PCD.close();

    std::cout << "File saved as newTest.pcd" << std::endl;
  }
}

/*=====================Helper Functions====================*/
void Robot::print_1dVec(std::vector<double> vec) {
  std::cout << " =========== vector begins ========= " << std::endl;
  std::cout << "[ ";
  for (int j = 0; j < vec.size(); ++j) {
    std::cout << vec[j] << " ";
  }
  std::cout << "] ";
  std::cout << std::endl;
  std::cout << " ============ vector ends ========== " << std::endl;
}

void Robot::print_2dVec(std::vector<std::vector<double>> vec) {
  std::cout << " =========== vector begins ========= " << std::endl;
  for (int i = 0; i < vec.size(); ++i) {
    std::cout << "[ ";
    for (int j = 0; j < vec[i].size(); ++j) {
      std::cout << vec[i][j] << " ";
    }
    std::cout << "] ";
    std::cout << std::endl;
  }
  std::cout << " ============ vector ends ========== " << std::endl;
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
