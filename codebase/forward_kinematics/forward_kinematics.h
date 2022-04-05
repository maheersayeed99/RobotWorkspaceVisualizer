#include <Eigen/Dense>
#include <iostream>

Robot::forward_kinematics(linked_list<Joint> joints) {
    Joint* next_joint = joints[1];

    while (nullptr != next_joint) {
        Eigen::Matrix<double, 1, 6> twist;

        Eigen::Matrix<double, 4, 4> transformation_matrix;
        transformation_matrix << 1, 0, 0, next_joint->origin_xyz[0],
                                0, 1, 0, next_joint->origin_xyz[1],
                                0, 0, 1, next_joint->origin_xyz[2],
                                0, 0, 0, 1;
        // loaded identity matrix, plus translation vector v


    }
}

