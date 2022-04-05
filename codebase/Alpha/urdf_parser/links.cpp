/* REFERENCE: https://github.com/ros/urdfdom/tree/master/urdf_parser --> Author: Wim Meeussen
Our team is implementing a simplified version of the source code above */
/* We only consider kinematic properties */

#include <vector>
#include <tinyxml.h>

namespace urdf{

    // =================================== FUNCTIONS FOR LINKS ===================================

    bool parsePose(Pose &pose, TiXmlElement* xml);

    bool parseCylinder(Cylinder &y, TiXmlElement *c) //cylindrical links only
    {
    return true;
    }

    bool parseLink(Link &link, TiXmlElement* config)
    {
    return true;
    }
}