#include "Robot.h"
/*=====================Helper Functions====================*/
// Utility functions
Eigen::Matrix3d Robot::Euler_to_Rmatrix(Eigen::Vector3d rpy) const {
  // assume axis is x_y_z
  Eigen::Matrix3d RM;
  double r = rpy(0);
  double p = rpy(1);
  double y = rpy(2);
  RM << cos(r) * cos(p) * cos(y) - sin(r) * sin(y),
      -cos(r) * cos(p) * sin(y) - sin(r) * cos(y), cos(r) * sin(p),
      sin(r) * cos(p) * cos(y) + cos(r) * sin(y),
      -sin(r) * cos(p) * sin(y) + cos(r) * cos(y), sin(r) * sin(p),
      -sin(p) * cos(y), sin(p) * sin(y), cos(p);

  return RM;
}
Eigen::Vector3d Robot::Rmatrix_to_Euler(Eigen::Matrix3d RM) const {
  double p = acos(RM(8));  // acos of last element is p
  // TODO: Fix this
  double y = asin(RM(7) / sin(p));
  double r = acos(RM(2) / sin(p));

  Eigen::Vector3d rpy(r, p, y);
  return rpy;
}

void string2vector3d(Eigen::Vector3d& vec, const char* stringPtr) {
  std::stringstream ss;
  double tempx, tempy, tempz;
  ss.clear();
  ss << stringPtr;
  ss >> tempx;
  ss >> tempy;
  ss >> tempz;
  vec << tempx, tempy, tempz;
}

/*=====================Main code====================*/
Robot::Robot() { ; }

int Robot::read_urdf(std::string fileName) {
  // Read URDF file
  tinyxml2::XMLDocument xmlDoc;
  tinyxml2::XMLError eResult = xmlDoc.LoadFile(fileName.c_str());
  XMLCheckResult(eResult);

  // Check if world exist
  tinyxml2::XMLNode* pRoot = xmlDoc.FirstChild();
  if (pRoot == nullptr) return tinyxml2::XML_ERROR_FILE_READ_ERROR;

  tinyxml2::XMLElement* pListElement = pRoot->FirstChildElement("joint");
  if (pListElement == nullptr) return tinyxml2::XML_ERROR_PARSING_ELEMENT;

  while (pListElement != nullptr) {
    Robot::Joint joint;

    // TODO: Use default value if a non-crucial domain does not exist

    // ################# JOINT NAME & TYPE ################# //
    const char* name = nullptr;
    const char* type = nullptr;
    eResult = pListElement->QueryStringAttribute("name", &name);
    XMLCheckResult(eResult);
    eResult = pListElement->QueryStringAttribute("type", &type);
    XMLCheckResult(eResult);

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

    // ################# PARENT LINK ################# //
    tinyxml2::XMLElement* pElement = pListElement->FirstChildElement("parent");
    if (pElement == nullptr) return tinyxml2::XML_ERROR_PARSING_ELEMENT;
    const char* parent = nullptr;
    eResult = pElement->QueryStringAttribute("link", &parent);
    XMLCheckResult(eResult);
    // printf("%s\n", parent);
    joint.parent_link_name_ = std::string(parent);

    // ################# CHILD LINK ################# //
    pElement = pListElement->FirstChildElement("child");
    if (pElement == nullptr) return tinyxml2::XML_ERROR_PARSING_ELEMENT;
    const char* child = nullptr;
    eResult = pElement->QueryStringAttribute("link", &child);
    XMLCheckResult(eResult);
    // printf("%s\n", child);
    joint.child_link_name_ = std::string(child);

    // ################# FRAME WRT CHILD ################# //
    pElement = pListElement->FirstChildElement("origin");
    if (pElement == nullptr) return tinyxml2::XML_ERROR_PARSING_ELEMENT;
    const char* rpy = nullptr;
    const char* xyz_pos = nullptr;
    eResult = pElement->QueryStringAttribute("rpy", &rpy);
    XMLCheckResult(eResult);
    eResult = pElement->QueryStringAttribute("xyz", &xyz_pos);
    XMLCheckResult(eResult);
    string2vector3d(joint.origin_rpy_, rpy);
    string2vector3d(joint.origin_xyz_, xyz_pos);

    // ################# AXIS ################# //
    pElement = pListElement->FirstChildElement("axis");
    if (pElement == nullptr) return tinyxml2::XML_ERROR_PARSING_ELEMENT;
    const char* xyz_axis = nullptr;
    eResult = pElement->QueryStringAttribute("xyz", &xyz_axis);
    XMLCheckResult(eResult);
    string2vector3d(joint.axis_, xyz_axis);

    // ################# JOINT LIMIT ################# //
    pElement = pListElement->FirstChildElement("limit");
    if (pElement == nullptr) return tinyxml2::XML_ERROR_PARSING_ELEMENT;
    const char* lower = nullptr;
    const char* upper = nullptr;
    eResult = pElement->QueryStringAttribute("lower", &lower);
    XMLCheckResult(eResult);
    eResult = pElement->QueryStringAttribute("upper", &upper);
    XMLCheckResult(eResult);
    joint.joint_limits_[0] = std::stod(lower);
    joint.joint_limits_[1] = std::stod(upper);

    // ################# SAVE ################# //
    joints_.push_back(joint);
    pListElement = pListElement->NextSiblingElement("joint");
  }
  // TODO: Check URDF structure
  // ##################### SUMMARY ##################### //
  for (auto joint : joints_) {
    std::cout << "Joint name: " << joint.joint_name_ << std::endl;
    std::cout << "Joint type: " << joint.joint_type_ << std::endl;
    std::cout << "Joint parent: " << joint.parent_link_name_ << std::endl;
    std::cout << "Joint child: " << joint.child_link_name_ << std::endl;
    // std::cout << "Joint origin rpy: \n" << joint.origin_rpy_ << std::endl;
    // std::cout << "Joint origin xyz: \n" << joint.origin_xyz_ << std::endl;
    // std::cout << "Joint axis xyz: \n" << joint.axis_ << std::endl;
    std::cout << std::endl;
  }
  return 0;
}

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

    for (auto const& pair : point_cloud_) {
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

Eigen::Vector3d Robot::forward_kinematics_single(
    std::vector<double> joint_angle) const {
  std::vector<Eigen::Matrix4d> gsts;

  // tool and base frame transformation
  Eigen::Vector3d tool_translate(0, 0, 0);

  int angle_index = 0;
  for (Joint next_joint : joints_) {
    double angle = joint_angle.at(angle_index);
    angle_index++;

    // handle rotation rpy
    Eigen::Matrix3d RMrpy = Euler_to_Rmatrix(next_joint.origin_rpy_);

    // update tool-base frame transform
    tool_translate = tool_translate + next_joint.origin_xyz_;

    // translation vector v = -1*cross(w, q)
    // w is axis, q is origin_xyz
    Eigen::Vector3d v;
    v = -1 * next_joint.axis_.cross(next_joint.origin_xyz_);
    Eigen::Vector3d w;
    w << next_joint.axis_;

    // fill twist hat vector
    Eigen::Matrix4d twist_hat;
    twist_hat << 0, -1 * w(2), w(1), v(0), w(2), 0, -1 * w(0), v(1), -1 * w(1),
        w(0), 0;
    v(2), 0, 0, 0, 0;

    // initialize transformation matrix g_ij()
    Eigen::Matrix4d transformation_matrix;

    Eigen::Matrix4d twist_by_angle;
    twist_by_angle = twist_hat * angle;

    // g_ij = e ^ [twist_hat * theta]
    transformation_matrix = twist_by_angle.exp();

    // add to sequence of transforms
    gsts.push_back(transformation_matrix);
  }

  // full matrix transform
  Eigen::Matrix4d total_transform = gsts.at(0);

  for (int j = 1; j < gsts.size(); j++) {
    total_transform = total_transform * gsts.at(j);
  }
  // full matrix transform "total_transform" complete

  // transformation between tool and base frame at theta=0
  Eigen::Matrix4d gs0;
  gs0 << 1, 0, 0, tool_translate(0), 0, 1, 0, tool_translate(1), 0, 0, 1,
      tool_translate(2), 0, 0, 0, 1;

  // TODO: tool_translate need to be a Vector4d
  Eigen::Matrix4d toolFrame;
  toolFrame = total_transform * gs0;

  Eigen::Matrix3d finalRotation;
  finalRotation << toolFrame(0), toolFrame(1), toolFrame(2), toolFrame(4),
      toolFrame(5), toolFrame(6), toolFrame(8), toolFrame(9), toolFrame(10);

  // FINAL POSITION
  Eigen::Vector3d final_position(toolFrame(3), toolFrame(7), toolFrame(11));

  // FINAL ROTATION AROUND axis_ = [1, 1, 1];
  Eigen::Vector3d final_rotation;
  final_rotation = Rmatrix_to_Euler(finalRotation);

  // FINAL AXIS
  Eigen::Vector3d final_axis(1, 1, 1);

  return final_position;
}

// Testing only, do not use in final version
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
                       hashFunction> const& m) {
  for (auto const& pair : m) {
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


void Robot::makePCD() {

    for (auto const& pair : point_cloud_) {

        vtx.push_back((float)pair.first[0]);
        vtx.push_back((float)pair.first[1]);
        vtx.push_back((float)pair.first[2]);

        col.push_back(1);
        col.push_back(1);
        col.push_back(0);
        col.push_back(1);

    }

}

void Robot::makeTempPCD(int numPoints) {
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