/* REFERENCE: https://github.com/ros/urdfdom/tree/master/urdf_parser --> Author: John Hsu
Our team is implementing a simplified version of the source code above */
/* We only consider kinematic properties */

#include <tinyxml.h>

namespace urdf{

    // =================================== FUNCTIONS FOR JOINTS ===================================

    bool parsePose(Pose &pose, TiXmlElement* xml);

    bool parseJointLimits(JointLimits &jl, TiXmlElement* config)
    {
    return true;
    }

    bool parseJoint(Joint &joint, TiXmlElement* config) //prismatic or revolute ONLY
    {
    return true;
    }
}