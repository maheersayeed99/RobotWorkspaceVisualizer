#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>  //which contains the required definitions to load and store point clouds to PCD and other file formats.

main(int argc, char **argv) {
  ros::init(argc, argv, "PCD_viewer");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub =
      nh.advertise<sensor_msgs::PointCloud2>("pcl_output1", 1);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;
  std::string pcd_path;
  nh.getParam("pcd_path", pcd_path);
  pcl::io::loadPCDFile(pcd_path, cloud);
  // Convert the cloud to ROS message
  pcl::toROSMsg(cloud, output);
  output.header.frame_id =
      "map";  // this has been done in order to be able to visualize our
              // PointCloud2 message on the RViz visualizer
  ros::Rate loop_rate(1);
  while (ros::ok()) {
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}