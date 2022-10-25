#pragma once

#include <Eigen/Dense>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>

#include "rpg_common/cv_type.h"

namespace rpg_common {

// In contrast to CV's eigen conversion, this has hard checks that also compile
// in Release mode.
template <typename Type, int Rows, int Cols>
Eigen::Matrix<Type, Rows, Cols> cvToEigen(const cv::Mat& mat)
{
  if (Rows != Eigen::Dynamic) CHECK_EQ(mat.rows, Rows);
  if (Cols != Eigen::Dynamic) CHECK_EQ(mat.cols, Cols);
  CHECK_EQ(mat.type(), CvType<Type>::value);
  return Eigen::Map<Eigen::Matrix<Type, Rows, Cols, Eigen::RowMajor>>(
      reinterpret_cast<Type*>(mat.data), mat.rows, mat.cols);
}

// Assumes each vector element is a column of the result.
template <typename Type>
void cvToEigen(const std::vector<cv::Mat>& mats,
               Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>* result)
{
  CHECK_NOTNULL(result);

  if (mats.empty())
  {
    result->resize(0, 0);
    return;
  }

  const int rows = mats[0].rows;
  result->resize(rows, mats.size());

  for (size_t i = 0u; i < mats.size(); ++i)
  {
    CHECK_EQ(mats[i].rows, rows);
    CHECK_EQ(mats[i].cols, 1);
    CHECK_EQ(mats[i].type(), CvType<Type>::value);
    result->col(i) = Eigen::Map<Eigen::Matrix<Type, Eigen::Dynamic, 1>>(
        reinterpret_cast<Type*>(mats[i].data), mats[i].rows);
  }
}

}  // namespace rpg_common
namespace rpg = rpg_common;
