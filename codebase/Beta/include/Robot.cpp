#include "Robot.h"
/*=====================Helper Functions====================*/
// Utility functions

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

void Robot::print_joints() const {
  for (auto joint : joints_) {
    std::cout << "Joint name: " << joint.joint_name_ << std::endl;
    std::cout << "Joint type: " << joint.joint_type_ << std::endl;
    std::cout << "Joint parent: " << joint.parent_link_name_ << std::endl;
    std::cout << "Joint child: " << joint.child_link_name_ << std::endl;
    std::cout << "Joint origin rpy: " << joint.origin_rpy_ << std::endl;
    std::cout << "Joint origin xyz: " << joint.origin_xyz_ << std::endl;
    std::cout << "Joint axis xyz: " << joint.axis_ << std::endl;
    std::cout << "Joint limit lower: " << joint.joint_limits_[0] << std::endl;
    std::cout << "Joint limit upper: " << joint.joint_limits_[1] << std::endl;
    std::cout << std::endl;
  }
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
  // for (auto joint : joints_) {
  //   std::cout << "Joint name: " << joint.joint_name_ << std::endl;
  //   std::cout << "Joint type: " << joint.joint_type_ << std::endl;
  //   std::cout << "Joint parent: " << joint.parent_link_name_ << std::endl;
  //   std::cout << "Joint child: " << joint.child_link_name_ << std::endl;
  //   // std::cout << "Joint origin rpy: \n" << joint.origin_rpy_ << std::endl;
  //   // std::cout << "Joint origin xyz: \n" << joint.origin_xyz_ << std::endl;
  //   // std::cout << "Joint axis xyz: \n" << joint.axis_ << std::endl;
  //   std::cout << std::endl;
  // }
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
  Eigen::Vector3d base2joint_pos(0, 0, 0);
  int angle_index = 0;

  for (auto joint : joints_) {
    // q
    base2joint_pos += joint.origin_xyz_;

    if (!std::strcmp(joint.joint_name_.c_str(), "world_joint") ||
        !std::strcmp(joint.joint_name_.c_str(), "ee_joint")) {
      continue;
    }

    double angle = joint_angle.at(angle_index);
    angle_index++;

    // store w
    Eigen::Vector3d w;
    w << joint.axis_;

    // compute v
    Eigen::Vector3d v;
    v = -1 * joint.axis_.cross(base2joint_pos);

    // xi_hat
    Eigen::Matrix4d twist_hat;
    twist_hat << 0, -1 * w(2), w(1), v(0), w(2), 0, -1 * w(0), v(1), -1 * w(1),
        w(0), 0, v(2), 0, 0, 0, 0;

    // g_ij
    Eigen::Matrix4d transformation_matrix;
    transformation_matrix = (twist_hat * angle).exp();

    gsts.push_back(transformation_matrix);
  }

  Eigen::Matrix4d base2joint_transform = gsts.at(0);
  for (int j = 1; j < gsts.size(); j++) {
    base2joint_transform = base2joint_transform * gsts.at(j);
  }

  Eigen::Matrix4d gs0;
  gs0 << 1, 0, 0, base2joint_pos(0), 0, 1, 0, base2joint_pos(1), 0, 0, 1,
      base2joint_pos(2), 0, 0, 0, 1;

  // TODO: tool_translate need to be a Vector4d
  Eigen::Matrix4d base2ee_transform;
  base2ee_transform = base2joint_transform * gs0;

  Eigen::Vector3d eeFrame_pos;
  eeFrame_pos << base2ee_transform(12), base2ee_transform(13),
      base2ee_transform(14);

  return eeFrame_pos;
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