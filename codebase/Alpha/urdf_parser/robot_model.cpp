/* REFERENCE: https://github.com/ros/urdfdom/tree/master/urdf_parser --> Author: Wim Meeussen
Our team is implementing a simplified version of the source code above */
/* We only consider kinematic properties */

#include <tinyxml.h>

namespace urdf{

  // =================================== GET LINK & JOINT PROPERTIES & STORE IN DATA STRUCT ===================================

  bool parseLink(Link &link, TiXmlElement *config);
  bool parseJoint(Joint &joint, TiXmlElement *config);

}