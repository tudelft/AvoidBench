#include "rpg_common/triangulate.h"

#include <eigen-checks/gtest.h>

#include "rpg_common/test_main.h"

class TriangulateFixture : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    B_.getPosition() << 1, 0, 0;
    Eigen::Matrix3d R;
    R << 1/sqrt(2.), 0, -1/sqrt(2.), 0, 1, 0, 1/sqrt(2.), 0, 1/sqrt(2.);
    B_.getRotation() = rpg::Pose::Rotation(R);

    a_.resize(3, 1);

    a_ << 0,0,1;
    b_ = a_;
  }

  rpg::Pose A_, B_;
  Eigen::Matrix3Xd a_, b_;
};

TEST_F(TriangulateFixture, Basic)
{
  Eigen::Matrix3Xd c;
  EXPECT_TRUE(rpg::triangulate(A_.inverse(), B_.inverse(), a_, b_, &c));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(c.col(0), Eigen::Vector3d::UnitZ()));
}

TEST_F(TriangulateFixture, Behind)
{
  B_.getRotation() = B_.getRotation().inverse();
  Eigen::Matrix3Xd c;
  EXPECT_FALSE(rpg::triangulate(A_.inverse(), B_.inverse(), a_, b_, &c));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(c.col(0), -Eigen::Vector3d::UnitZ()));
}

TEST_F(TriangulateFixture, Parallel)
{
  B_.getRotation().setIdentity();
  Eigen::Matrix3Xd c;
  EXPECT_FALSE(rpg::triangulate(A_.inverse(), B_.inverse(), a_, b_, &c));
  EXPECT_TRUE(std::isnan(c(0, 0)));
  EXPECT_TRUE(std::isnan(c(1, 0)));
  EXPECT_TRUE(std::isnan(c(2, 0)));
}

RPG_COMMON_TEST_MAIN
{}
