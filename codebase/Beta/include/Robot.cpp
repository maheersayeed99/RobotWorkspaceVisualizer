#include "Robot.h"

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
    std::cout << "Joint origin rpy: \n" << joint.origin_rpy_ << std::endl;
    std::cout << "Joint origin xyz: \n" << joint.origin_xyz_ << std::endl;
    std::cout << "Joint axis xyz: \n" << joint.axis_ << std::endl;
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
