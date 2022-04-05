#ifndef ROBOT_IS_INCLUDED
#define ROBOT_IS_INCLUDED

#include <Eigen/Dense>
#include <vector>
#include <unordered_map>

#include <pcl/io/pcd_io.h>                  // include libraries
#include <pcl/point_types.h>


class Robot {
 public:
  Robot();

  class Joint {
    friend Robot;
    std::string frame_name_;
    std::string parent_name_;
    Eigen::Vector3d origin_xyz_;
    Eigen::Vector3d origin_rpy_;
    Eigen::Vector3d axis_;
    double joint_state_;
    double joint_limits_[2];
  };
  bool read_urdf(std::string filepath);
  void forward_kinematics(std::vector<Joint> joints,
                          std::vector<double> joint_angle);
  void get_workspace();
  bool save2pcd(std::string filepath);

 private:
  std::vector<Joint> joints_;
  std::unordered_map<std::vector<double>, std::vector<std::vector<double>>>
      point_cloud_;
};


bool Robot::save2pcd(std::string filepath) {

    pcl::PointCloud<pcl::PointXYZ> cloud;           // make point cloud object

    cloud.points.resize(joints_.size());            // resize cloud

    int i = 0;
    for (auto& point : cloud)
    {
        point.x = joints_[i].x();                   // fill cloud
        point.y = joints_[i].y();
        point.z = joints_[i].z();

        i++;
    }

    if (pcl::io::savePCDFileASCII(filepath, cloud)) {       // save to file
        return 1;
    }
    else {
        return 0;
    }

   
}

#endif
