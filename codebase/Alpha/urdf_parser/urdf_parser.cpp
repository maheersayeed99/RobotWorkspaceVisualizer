#include "tinyxml2.h"
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <iostream>

using namespace tinyxml2;

#ifndef XMLCheckResult
#define XMLCheckResult(a_eResult)     \
  if (a_eResult != XML_SUCCESS) {     \
    printf("Error: %i\n", a_eResult); \
    return a_eResult;                 \
  }
#endif

class Robot {
 public:
  Robot();
  class Joint {
    friend Robot;

   public:
    typedef enum { fixed, revolute, continuous, prismatic } joint_type;
    joint_type joint_type_;
    std::string joint_name_;
    std::string child_link_name_;
    std::string parent_link_name_;
    Eigen::Vector3d origin_xyz_;
    Eigen::Vector3d origin_rpy_;
    Eigen::Vector3d axis_;
    double joint_state_;      // default angle of the joint
    double joint_limits_[2];  // need to implement
  };

 public:
  std::vector<Joint> joints_;
};
Robot::Robot() { ; }

int main(int argc, char* argv[]) {
  XMLDocument xmlDoc;
  Robot robot;

  XMLError eResult = xmlDoc.LoadFile("ur5.urdf");
  XMLCheckResult(eResult);

  XMLNode* pRoot = xmlDoc.FirstChild();
  if (pRoot == nullptr) return XML_ERROR_FILE_READ_ERROR;

  XMLElement* pListElement = pRoot->FirstChildElement("joint");
  if (pListElement == nullptr) return XML_ERROR_PARSING_ELEMENT;

  while (pListElement != nullptr) {
    Robot::Joint joint;

    // ########################################## JOINT NAME & TYPE
    // ########################################## //
    const char* name = nullptr;
    const char* type = nullptr;
    eResult = pListElement->QueryStringAttribute("name", &name);
    XMLCheckResult(eResult);
    eResult = pListElement->QueryStringAttribute("type", &type);
    XMLCheckResult(eResult);
    // printf("%s\n", name);
    // printf("%s\n", type);
    joint.joint_name_ = std::string(name);
    if (!std::strcmp(type, "fixed")) {
      joint.joint_type_ = joint.fixed;
    } else if (!std::strcmp(type, "revolute")) {
      joint.joint_type_ = joint.revolute;
    } else if (!std::strcmp(type, "continuous")) {
      joint.joint_type_ = joint.continuous;
    } else if (!std::strcmp(type, "prismatic")) {
      joint.joint_type_ = joint.prismatic;
    }

    // ########################################## PARENT LINK
    // ########################################## //
    XMLElement* pElement = pListElement->FirstChildElement("parent");
    if (pElement == nullptr) return XML_ERROR_PARSING_ELEMENT;
    const char* parent = nullptr;
    eResult = pElement->QueryStringAttribute("link", &parent);
    XMLCheckResult(eResult);
    // printf("%s\n", parent);
    joint.parent_link_name_ = std::string(parent);

    // ########################################## CHILD LINK
    // ########################################## //
    pElement = pListElement->FirstChildElement("child");
    if (pElement == nullptr) return XML_ERROR_PARSING_ELEMENT;
    const char* child = nullptr;
    eResult = pElement->QueryStringAttribute("link", &child);
    XMLCheckResult(eResult);
    // printf("%s\n", child);
    joint.child_link_name_ = std::string(child);

    // ########################################## FRAME WRT CHILD
    // ########################################## //
    pElement = pListElement->FirstChildElement("origin");
    if (pElement == nullptr) return XML_ERROR_PARSING_ELEMENT;
    const char* rpy = nullptr;
    const char* xyz_pos = nullptr;
    eResult = pElement->QueryStringAttribute("rpy", &rpy);
    XMLCheckResult(eResult);
    eResult = pElement->QueryStringAttribute("xyz", &xyz_pos);
    XMLCheckResult(eResult);
    // printf("%s\n", rpy);
    // printf("%s\n", xyz_pos);
    joint.origin_rpy_ << rpy[0], rpy[1], rpy[2];
    joint.origin_xyz_ << xyz_pos[0], xyz_pos[1], xyz_pos[2];

    // ########################################## AXIS
    // ########################################## //
    pElement = pListElement->FirstChildElement("axis");
    if (pElement == nullptr) return XML_ERROR_PARSING_ELEMENT;
    const char* xyz_axis = nullptr;
    eResult = pElement->QueryStringAttribute("xyz", &xyz_axis);
    XMLCheckResult(eResult);
    // printf("%s\n", xyz_axis);
    joint.axis_ << xyz_axis[0], xyz_axis[1], xyz_axis[2];

    robot.joints_.push_back(joint);
    pListElement = pListElement->NextSiblingElement("joint");
  }

  // ########################################## SUMMARY
  // ########################################## //
  for (auto joint : robot.joints_) {
    std::cout << "Joint name: " << joint.joint_name_ << std::endl;
    std::cout << "Joint type: " << joint.joint_type_ << std::endl;
    std::cout << "Joint parent: " << joint.parent_link_name_ << std::endl;
    std::cout << "Joint child: " << joint.child_link_name_ << std::endl;
    std::cout << "Joint origin rpy: " << joint.origin_rpy_ << std::endl;
    std::cout << "Joint origin xyz: " << joint.origin_xyz_ << std::endl;
    std::cout << "Joint axis xyz: " << joint.axis_ << std::endl;
    std::cout << std::endl;
  }
  return 0;
}
