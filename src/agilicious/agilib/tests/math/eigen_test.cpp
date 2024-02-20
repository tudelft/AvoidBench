#include <gtest/gtest.h>

#include "agilib/math/math.hpp"
#include "agilib/math/types.hpp"

using namespace agi;

TEST(EigenChecks, EigenVersionOutput) {
  std::printf("Eigen Version: %d.%d.%d\n", EIGEN_WORLD_VERSION,
              EIGEN_MAJOR_VERSION, EIGEN_MINOR_VERSION);
}

TEST(EigenChecks, EigenQuaternionSequence) {
  for (int i = 0; i < 1024; i++) {
    Quaternion q1, q2;

    q1.coeffs() = Vector<4>::Random();
    q1.normalize();
    q2.coeffs() = Vector<4>::Random();
    q2.normalize();

    const Matrix<3, 3> R1 = q1.toRotationMatrix();
    const Matrix<3, 3> R2 = q2.toRotationMatrix();

    // const Quaternion q3 = q2 * q1;
    const Quaternion q3 = q2 * q1;
    const Matrix<3, 3> Rq3 = q3.toRotationMatrix();

    const Matrix<3, 3> R3 = R2 * R1;

    EXPECT_TRUE(R3.isApprox(Rq3))
      << "Quternion multiplication sequence does not follow rotation!";

    const Vector<3> r = Vector<3>::Random();
    const Vector<3> r_R3 = R3 * r;

    const Vector<3> r_Rq3 = q3.toRotationMatrix() * r;
    EXPECT_TRUE(r_Rq3.isApprox(r_R3));

    const Vector<3> r_q3 = q3 * r;
    EXPECT_TRUE(r_q3.isApprox(r_R3));

    const Vector<3> r_q21 = (q2 * q1) * r;
    EXPECT_TRUE(r_q21.isApprox(r_R3));

    const Vector<3> r_q2q1 = q2 * (q1 * r);
    EXPECT_TRUE(r_q2q1.isApprox(r_R3));

    const Vector<3> r_R2R1 = R2 * R1 * r;
    EXPECT_TRUE(r_R2R1.isApprox(r_R3));
  }
}

TEST(EigenChecks, EigenQuaternionRotationDirection) {
  const Quaternion qz90(sqrt(0.5), 0, 0, sqrt(0.5));
  const Quaternion qy90(sqrt(0.5), 0, sqrt(0.5), 0);
  const Quaternion qz90y90 = qz90 * qy90;
  const Matrix<3, 3> Rz90 =
    (Matrix<3, 3>() << 0, -1, 0, 1, 0, 0, 0, 0, 1).finished();
  const Matrix<3, 3> Ry90 =
    (Matrix<3, 3>() << 0, 0, 1, 0, 1, 0, -1, 0, 0).finished();
  const Matrix<3, 3> Rz90y90 = Rz90 * Ry90;

  const Vector<3> r1(1, 0, 0);
  const Vector<3> r1_Rz90(0, -1, 0);
  const Vector<3> r1_Ry90(0, 0, 1);
  const Vector<3> r1_Rz90y90(0, -1, 0);

  EXPECT_TRUE(qz90.toRotationMatrix().isApprox(Rz90));
  EXPECT_TRUE(qy90.toRotationMatrix().isApprox(Ry90));
  EXPECT_TRUE(qz90y90.toRotationMatrix().isApprox(Rz90y90));

  EXPECT_TRUE((Rz90 * r1_Rz90).isApprox(r1));
  EXPECT_TRUE((Ry90 * r1_Ry90).isApprox(r1));
  EXPECT_TRUE((qz90.toRotationMatrix() * r1_Rz90).isApprox(r1));
  EXPECT_TRUE((qz90 * r1_Rz90).isApprox(r1));
  EXPECT_TRUE((qy90.toRotationMatrix() * r1_Ry90).isApprox(r1));
  EXPECT_TRUE((qy90 * r1_Ry90).isApprox(r1));

  EXPECT_TRUE((Rz90y90 * r1_Rz90y90).isApprox(r1));
  EXPECT_TRUE((qz90y90.toRotationMatrix() * r1_Rz90y90).isApprox(r1));
  EXPECT_TRUE((qz90y90 * r1_Rz90y90).isApprox(r1));

  const Vector<3> r2(0, 1, 0);
  const Vector<3> r2_Rz90(1, 0, 0);
  const Vector<3> r2_Ry90(0, 1, 0);
  const Vector<3> r2_Rz90y90(0, 0, 1);

  EXPECT_TRUE((Rz90 * r2_Rz90).isApprox(r2));
  EXPECT_TRUE((Ry90 * r2_Ry90).isApprox(r2));
  EXPECT_TRUE((qz90.toRotationMatrix() * r2_Rz90).isApprox(r2));
  EXPECT_TRUE((qz90 * r2_Rz90).isApprox(r2));
  EXPECT_TRUE((qy90.toRotationMatrix() * r2_Ry90).isApprox(r2));
  EXPECT_TRUE((qy90 * r2_Ry90).isApprox(r2));

  EXPECT_TRUE((Rz90y90 * r2_Rz90y90).isApprox(r2));
  EXPECT_TRUE((qz90y90.toRotationMatrix() * r2_Rz90y90).isApprox(r2));
  EXPECT_TRUE((qz90y90 * r2_Rz90y90).isApprox(r2));
}

TEST(EigenChecks, QuaternionCrossMatrix) {
  const Vector<4> v1 = Vector<4>::Random().normalized();
  const Vector<4> v2 = Vector<4>::Random().normalized();

  EXPECT_NEAR(v1.norm(), 1.0, 1e-9);
  EXPECT_NEAR(v2.norm(), 1.0, 1e-9);

  const Quaternion q1(v1(0), v1(1), v1(2), v1(3));
  const Quaternion q2(v2(0), v2(1), v2(2), v2(3));

  const Quaternion q1q2 = q1 * q2;
  const Quaternion q2q1 = q2 * q1;

  const Vector<4> vq1q2 = (Vector<4>() << q1q2.w(), q1q2.vec()).finished();
  const Vector<4> vq2q1 = (Vector<4>() << q2q1.w(), q2q1.vec()).finished();

  const Vector<4> Qleftq1_v2 = Q_left(q1) * v2;
  const Vector<4> Qrightq2_v1 = Q_right(q2) * v1;

  const Vector<4> Qleftq2_v1 = Q_left(q2) * v1;
  const Vector<4> Qrightq1_v2 = Q_right(q1) * v2;

  EXPECT_TRUE(Qleftq1_v2.isApprox(vq1q2));
  EXPECT_TRUE(Qrightq2_v1.isApprox(vq1q2));
  EXPECT_TRUE(Qleftq2_v1.isApprox(vq2q1));
  EXPECT_TRUE(Qrightq1_v2.isApprox(vq2q1));
}

TEST(EigenChecks, QuaternionDecomposition) {
  for (int i = 0; i < 1024; i++) {
    const Quaternion qe = Quaternion(Vector<4>::Random()).normalized();

    // We don't get this order right now.
    const Quaternion qz = Quaternion(qe.w(), 0, 0, qe.z()).normalized();

    const Quaternion qxy = qe * qz.inverse();

    EXPECT_NEAR(qe.norm(), 1.0, 1e-6);
    EXPECT_NEAR(qz.norm(), 1.0, 1e-6);
    EXPECT_NEAR(qxy.norm(), 1.0, 1e-6);

    const Vector<3> v_expected = qe.toRotationMatrix() * Vector<3>::UnitZ();
    const Vector<3> v_z_equivalent =
      qxy.toRotationMatrix() * Vector<3>::UnitZ();
    EXPECT_TRUE(v_z_equivalent.isApprox(v_expected, 1e-6))
      << "Expected: " << v_expected.transpose() << std::endl
      << "Actual:   " << v_z_equivalent.transpose() << std::endl
      << "q_xy:     " << qxy.coeffs().transpose() << std::endl;

    const Vector<3> v_z_invariant = qz.toRotationMatrix() * Vector<3>::UnitZ();
    EXPECT_TRUE(v_z_invariant.isApprox(Vector<3>::UnitZ(), 1e-6));
    const Quaternion q_xy_vec{
      Quaternion::FromTwoVectors(Vector<3>::UnitZ(), v_expected)};
    std::cout << "From Vectors:  " << q_xy_vec.coeffs().transpose()
              << std::endl;
  }
}

TEST(EigenChecks, MatrixColumnwiseDotProduct) {
  static constexpr size_t N = 64;
  static constexpr size_t S = 2;
  const Matrix<S, N> A = Matrix<S, N>::Random();

  Vector<N> expected = Vector<N>::Zero();
  for (size_t i = 0; i < N; ++i) expected(i) = A.col(i).transpose() * A.col(i);

  Vector<N> dotproduct = (A.cwiseProduct(A)).colwise().sum();

  EXPECT_TRUE(dotproduct.isApprox(expected));
}
