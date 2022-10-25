#include "rpg_common/triangulate.h"

#include <cmath>

#include <glog/logging.h>

#include "rpg_common/select.h"

namespace rpg_common {


bool triangulate(
    const Pose& T_A_W, const Pose& T_B_W, const Eigen::Matrix3Xd& bv_A,
    const Eigen::Matrix3Xd& bv_B, Eigen::Matrix3Xd* p_W)
{
  std::vector<bool> in_front_of_both;
  triangulate(T_A_W, T_B_W, bv_A, bv_B, p_W, &in_front_of_both);
  return !select(in_front_of_both).empty();
}

void triangulate(
    const Pose& T_A_W, const Pose& T_B_W, const Eigen::Matrix3Xd& bv_A,
    const Eigen::Matrix3Xd& bv_B, Eigen::Matrix3Xd* p_W,
    std::vector<bool>* in_front_of_both)
{
  const int N = bv_A.cols();
  CHECK_NOTNULL(p_W)->resize(Eigen::NoChange, N);
  CHECK_NOTNULL(in_front_of_both)->resize(N);
  CHECK_EQ(N, bv_B.cols());

  const Eigen::Matrix<double, 3, 4> M_A =
      T_A_W.getTransformationMatrix().topRows<3>();
  const Eigen::Matrix<double, 3, 4> M_B =
      T_B_W.getTransformationMatrix().topRows<3>();

  for (int i = 0; i < N; ++i)
  {
    Eigen::Matrix<double, 6, 4> A;
    A.topRows<3>() = -M_A.colwise().cross(bv_A.col(i));
    A.bottomRows<3>() = -M_B.colwise().cross(bv_B.col(i));
    const Eigen::JacobiSVD<Eigen::Matrix<double, 6, 4>> svd(
        A, Eigen::ComputeFullV);
    const Eigen::Vector4d p_W_i_4 = svd.matrixV().col(3);

    if (p_W_i_4(3) == 0.)
    {
      (*in_front_of_both)[i] = false;
      p_W->col(i).setConstant(NAN);
      continue;
    }

    const Eigen::Vector3d p_W_i = p_W_i_4.head<3>() / p_W_i_4(3);
    p_W->col(i) = p_W_i;
    (*in_front_of_both)[i] = (T_A_W * p_W_i)(2) > 0. && (T_B_W * p_W_i)(2) > 0.;
  }
}

bool triangulate(const Aligned<std::vector, Pose>& T_W_C,
                 const Aligned<std::vector, Eigen::Vector3d>& bv_C,
                 Eigen::Vector3d* p_W)
{
  const size_t n = T_W_C.size();
  CHECK_EQ(bv_C.size(), n);
  CHECK_NOTNULL(p_W);
  typedef Eigen::Matrix<double, Eigen::Dynamic, 4> AMatrix;
  AMatrix A(3 * n, 4);
  for (size_t i = 0u; i < n; ++i)
  {
    const Eigen::Matrix<double, 3, 4> M =
        T_W_C[i].inverse().getTransformationMatrix().topRows<3>();
    A.middleRows<3>(3 * i) = -M.colwise().cross(bv_C[i]);
  }

  const Eigen::JacobiSVD<AMatrix> svd(A, Eigen::ComputeFullV);
  const Eigen::Vector4d p_W_4 = svd.matrixV().col(3);

  if (p_W_4(3) == 0.)
  {
    p_W->setConstant(NAN);
    return false;
  }

  *p_W = p_W_4.head<3>() / p_W_4(3);
  return true;
}

}  // namespace rpg_common
