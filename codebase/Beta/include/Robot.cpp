#include "Robot.h"

Robot::Robot() { ; }

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
