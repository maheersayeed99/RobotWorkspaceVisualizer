/* REFERENCE: https://github.com/ros/urdfdom/tree/master/urdf_parser --> Author: Wim Meeussen, John Hsu
Our team is implementing a simplified version of the source code above */
/* We only consider kinematic properties */

#include <tinyxml.h>
#include <vector>

namespace urdf_export_helpers {

    // =================================== HELPER FUNCTIONS ===================================

    std::string values2str(urdf::Vector3 vec)
    {
        double xyz[3];
        xyz[0] = vec.x;
        xyz[1] = vec.y;
        xyz[2] = vec.z;
        return values2str(3, xyz);
    }

    std::string values2str(urdf::Rotation rot)
    {
        double rpy[3];
        rot.getRPY(rpy[0], rpy[1], rpy[2]);
        return values2str(3, rpy);
    }
}