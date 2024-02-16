#include <Eigen/Eigen>

#include "benchmark/benchmark.h"


template<typename T>
Eigen::Matrix<T, 4, 4> Q_left(const Eigen::Quaternion<T>& q) {
  return (Eigen::Matrix<T, 4, 4>() << q.w(), -q.x(), -q.y(), -q.z(), q.x(),
          q.w(), -q.z(), q.y(), q.y(), q.z(), q.w(), -q.x(), q.z(), -q.y(),
          q.x(), q.w())
    .finished();
}

template<typename Vector>
Eigen::Matrix<typename Vector::Scalar, 3, 3> skew(const Vector& v) {
  using T = typename Vector::Scalar;
  return (Eigen::Matrix<T, 3, 3>() << 0, -v.z(), v.y(), v.z(), 0, -v.x(),
          -v.y(), v.x(), 0)
    .finished();
}

template<typename T>
Eigen::Matrix<T, 4, 4> Q_left_seq(const Eigen::Quaternion<T>& q) {
  return q.w() * Eigen::Matrix<T, 4, 4>::Identity() +
         (Eigen::Matrix<T, 4, 4>() << 0, -q.vec().transpose(), q.vec(),
          skew(q.vec()))
           .finished();
}


template<typename MatrixType>
struct types {
  using Scalar = typename MatrixType::Scalar;

  template<int rows = Eigen::Dynamic>
  using Vector = std::conditional_t<std::is_const<MatrixType>::value,
                                    const Eigen::Matrix<Scalar, rows, 1>,
                                    Eigen::Matrix<Scalar, rows, 1>>;

  template<int rows = Eigen::Dynamic, int cols = Eigen::Dynamic>
  using Matrix = std::conditional_t<std::is_const<MatrixType>::value,
                                    const Eigen::Matrix<Scalar, rows, cols>,
                                    Eigen::Matrix<Scalar, rows, cols>>;

  using Quaternion = std::conditional_t<std::is_const<MatrixType>::value,
                                        const Eigen::Quaternion<Scalar>,
                                        Eigen::Quaternion<Scalar>>;
};

template<typename MT>
void directQleftStatic(benchmark::State& bench) {
  using Vector4 = typename types<MT>::template Vector<4>;
  using Matrix4 = typename types<MT>::template Matrix<4, 4>;
  using Quaternion = typename types<MT>::Quaternion;

  Vector4 qvec = Vector4::Random();
  Quaternion q =
    Quaternion(qvec.w(), qvec.x(), qvec.y(), qvec.z()).normalized();

  for (auto _ : bench) {
    Matrix4 Q = Q_left(q);
    benchmark::DoNotOptimize(Q);
  }
}
BENCHMARK_TEMPLATE1(directQleftStatic, Eigen::Matrix4d);
BENCHMARK_TEMPLATE1(directQleftStatic, const Eigen::Matrix4d);

template<typename MT>
void directQleftDynamic(benchmark::State& bench) {
  using Vector = typename types<MT>::template Vector<>;
  using Matrix = typename types<MT>::template Matrix<>;
  using Quaternion = typename types<MT>::Quaternion;

  Vector qvec = Vector::Random(4);
  Quaternion q =
    Quaternion(qvec.w(), qvec.x(), qvec.y(), qvec.z()).normalized();

  for (auto _ : bench) {
    Matrix Q = Q_left(q);
    benchmark::DoNotOptimize(Q);
  }
}
BENCHMARK_TEMPLATE1(directQleftDynamic, Eigen::Matrix4d);
BENCHMARK_TEMPLATE1(directQleftDynamic, const Eigen::Matrix4d);


template<typename MT>
void seqQleftStatic(benchmark::State& bench) {
  using Vector4 = typename types<MT>::template Vector<4>;
  using Matrix4 = typename types<MT>::template Matrix<4, 4>;
  using Quaternion = typename types<MT>::Quaternion;

  Vector4 qvec = Vector4::Random();
  Quaternion q =
    Quaternion(qvec.w(), qvec.x(), qvec.y(), qvec.z()).normalized();

  for (auto _ : bench) {
    Matrix4 Q = Q_left_seq(q);
    benchmark::DoNotOptimize(Q);
  }
}
BENCHMARK_TEMPLATE1(seqQleftStatic, Eigen::Matrix4d);
BENCHMARK_TEMPLATE1(seqQleftStatic, const Eigen::Matrix4d);

template<typename MT>
void seqQleftDynamic(benchmark::State& bench) {
  using Vector = typename types<MT>::template Vector<>;
  using Matrix = typename types<MT>::template Matrix<>;
  using Quaternion = typename types<MT>::Quaternion;

  Vector qvec = Vector::Random(4);
  Quaternion q =
    Quaternion(qvec.w(), qvec.x(), qvec.y(), qvec.z()).normalized();

  for (auto _ : bench) {
    Matrix Q = Q_left_seq(q);
    benchmark::DoNotOptimize(Q);
  }
}
BENCHMARK_TEMPLATE1(seqQleftDynamic, Eigen::Matrix4d);
BENCHMARK_TEMPLATE1(seqQleftDynamic, const Eigen::Matrix4d);