#include "rpg_common/cv_utils.h"

#include <eigen-checks/gtest.h>

#include "rpg_common/pose.h"
#include "rpg_common/test_main.h"

namespace rpg_common {

TEST(cv_utils, decomposeEssentialMatrix)
{
  Pose pose;
  pose.setRandom();  // Deterministic RNG.

  const Eigen::Matrix3d E =
      - pose.getRotationMatrix().colwise().cross(pose.getPosition());

  Eigen::Matrix3d R1, R2;
  Eigen::Vector3d t;
  cv_utils::decomposeEssentialMatrix(E, &R1, &R2, &t);

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(R1, pose.getRotationMatrix()) ||
              EIGEN_MATRIX_EQUAL_DOUBLE(R2, pose.getRotationMatrix()));

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(
      t.cross(pose.getPosition()), Eigen::Vector3d::Zero()));
}

}  // namespace rpg_common

RPG_COMMON_TEST_MAIN
{}
