/*
TEST FOUR:
Here we test whether the generate_config() function could produce 
correct number of combinations given number of joints and the user
defined resolution. Then we test whether the new forward kinematic
wrapper function can unpack results from forward_kinematics_single().
*/

#include "Robot.h"

int main(void)
{
    // fake inputs
    Robot robot;
    Robot::Joint joint0;
    joint0.joint_limits_[0] = 0.;
    joint0.joint_limits_[1] = 0.;
    Robot::Joint joint1;
    joint1.joint_limits_[0] = 0.;
    joint1.joint_limits_[1] = 2.;
    Robot::Joint joint2;
    joint2.joint_limits_[0] = -4.;
    joint2.joint_limits_[1] = 0.;
    Robot::Joint joint3;
    joint3.joint_limits_[0] = 10.;
    joint3.joint_limits_[1] = 14.;
    Robot::Joint joint4;
    joint4.joint_limits_[0] = 0.;
    joint4.joint_limits_[1] = 0.;
    robot.joints_.push_back(joint0);
    robot.joints_.push_back(joint1);
    robot.joints_.push_back(joint2);
    robot.joints_.push_back(joint3);
    robot.joints_.push_back(joint4);

    auto configs = robot.generate_config(3);
    assert(configs.size() == 3 * 3 * 3);
    robot.print_2dVec(configs);

    return 0;
}