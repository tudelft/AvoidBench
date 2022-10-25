#pragma once

#include <Eigen/Dense>

#include "rpg_common/aligned.h"
#include "rpg_common/pose.h"

namespace rpg_common {

// Triangulate corresponding bearing vectors from two frames. True only if all
// points lie in front of both cameras and no bearing vectors are parallel.
// p_W will be valid anyways (nan for parallel bearing vectors).
// Linear triangulation with SVD, see section 12.2 of Hartley and Zisserman's
// "Multiple View Geometry" book.
bool triangulate(
    const Pose& T_A_W, const Pose& T_B_W, const Eigen::Matrix3Xd& bv_A,
    const Eigen::Matrix3Xd& bv_B, Eigen::Matrix3Xd* p_W);
// Additionally returns for each point whether it lies in front of both cameras.
void triangulate(
    const Pose& T_A_W, const Pose& T_B_W, const Eigen::Matrix3Xd& bv_A,
    const Eigen::Matrix3Xd& bv_B, Eigen::Matrix3Xd* p_W,
    std::vector<bool>* in_front_of_both);

// Single-point triangulation from an arbitrary observation count.
bool triangulate(const Aligned<std::vector, Pose>& T_W_C,
                 const Aligned<std::vector, Eigen::Vector3d>& bv_C,
                 Eigen::Vector3d* p_W);

}  // namespace rpg_common
namespace rpg = rpg_common;
