
#ifndef ANALYTICAL_KINEMATICS_HPP_
#define ANALYTICAL_KINEMATICS_HPP_

#include <eigen3/Eigen/Dense>
#include <vector>

#include "rt_manipulators_cpp/kinematics_utils.hpp"
#include "rt_manipulators_cpp/link.hpp"

namespace analytical_kinematics {

void fk(const kinematics_utils::links_t & links, Eigen::Vector3d & eef_pos, Eigen::Matrix3d & eef_R);

}  // namespace analytical_kinematics

#endif  // ANALYTICAL_KINEMATICS_HPP_
