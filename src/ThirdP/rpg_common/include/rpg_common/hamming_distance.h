#pragma once

#include <Eigen/Dense>

namespace rpg_common {

int hammingDistance(
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, 1>& a,
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, 1>& b);

int hammingDistance(
    const Eigen::Matrix<
    unsigned char, Eigen::Dynamic, Eigen::Dynamic>::ConstColXpr& a,
    const Eigen::Matrix<
    unsigned char, Eigen::Dynamic, Eigen::Dynamic>::ConstColXpr& b);

}  // namespace rpg_common
namespace rpg = rpg_common;
