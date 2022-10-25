#pragma once

#include "rpg_common/aligned.h"

namespace rpg_common {

template <typename Scalar, int Rows, int Cols>
Aligned<std::vector, Eigen::Matrix<Scalar, Rows, 1>> colsVec(
    const Eigen::Matrix<Scalar, Rows, Cols>& input)
{
  Aligned<std::vector, Eigen::Matrix<Scalar, Rows, 1>> result;
  result.reserve(input.cols());
  for (int i = 0; i < input.cols(); ++i)
  {
    result.push_back(input.col(i));
  }
  return result;
}

}  // namespace rpg_common
namespace rpg = rpg_common;
