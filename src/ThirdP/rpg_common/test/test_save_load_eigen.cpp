#include "rpg_common/save.h"
#include "rpg_common/load.h"

#include <stdio.h>
#include <random>
#include <time.h>

#include <eigen-checks/gtest.h>

#include "rpg_common/test_main.h"

namespace rpg_common {

TEST(eigen_save_load, save_and_load)
{
  std::mt19937 gen(std::time(nullptr));
  std::uniform_int_distribution<> dis(2, 10);
  int size = dis(gen);

  const std::string fn("/tmp/save.txt");
  const Eigen::MatrixXd mat = Eigen::MatrixXd::Random(size, size);

  rpg::save(fn, mat, EigenSpaceSeparatedFmt);
  Eigen::MatrixXd load_mat;
  rpg::load(fn, &load_mat);
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(mat, load_mat));
  std::remove(fn.c_str());

  rpg::save(fn, mat, EigenCSVFmt);
  rpg::load(fn, &load_mat, ',');
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(mat, load_mat));
  std::remove(fn.c_str());
}

} // namespace rpg_common

RPG_COMMON_TEST_MAIN
{}
