#include <Eigen/Eigen>

#include "benchmark/benchmark.h"

using namespace Eigen;


void DynamicDenseDiagonalAssignmentEqual(benchmark::State& bench) {
  const size_t size = bench.range(0);

  const MatrixXd A = VectorXd::Random(size).asDiagonal();
  MatrixXd B = MatrixXd::Zero(size, size);

  for (auto _ : bench) {
    B = A;
    benchmark::DoNotOptimize(B);
  }

  bench.SetBytesProcessed(bench.iterations() * sizeof(double) * size * size);
}
BENCHMARK(DynamicDenseDiagonalAssignmentEqual)
  ->RangeMultiplier(2)
  ->Range(4, 256);

void DynamicDenseDiagonalAssignmentStream(benchmark::State& bench) {
  const size_t size = bench.range(0);

  const MatrixXd A = VectorXd::Random(size).asDiagonal();
  MatrixXd B = MatrixXd::Zero(size, size);

  for (auto _ : bench) {
    B << A;
    benchmark::DoNotOptimize(B);
  }

  bench.SetBytesProcessed(bench.iterations() * sizeof(double) * size * size);
}
BENCHMARK(DynamicDenseDiagonalAssignmentStream)
  ->RangeMultiplier(2)
  ->Range(4, 256);

void DynamicDenseDiagonalMultiply(benchmark::State& bench) {
  const size_t size = bench.range(0);

  const MatrixXd A = VectorXd::Random(size).asDiagonal();
  const MatrixXd B = VectorXd::Random(size).asDiagonal();
  MatrixXd C = MatrixXd::Zero(size, size);

  for (auto _ : bench) {
    C = A * B;
    benchmark::DoNotOptimize(C);
  }

  bench.SetBytesProcessed(bench.iterations() * sizeof(double) * size * size);
}
BENCHMARK(DynamicDenseDiagonalMultiply)->RangeMultiplier(2)->Range(4, 256);

void DynamicDenseNoAliasDiagonalMultiply(benchmark::State& bench) {
  const size_t size = bench.range(0);

  const MatrixXd A = VectorXd::Random(size).asDiagonal();
  const MatrixXd B = VectorXd::Random(size).asDiagonal();
  MatrixXd C = MatrixXd::Zero(size, size);

  for (auto _ : bench) {
    C.noalias() = A * B;
    benchmark::DoNotOptimize(C);
  }

  bench.SetBytesProcessed(bench.iterations() * sizeof(double) * size * size);
}
BENCHMARK(DynamicDenseNoAliasDiagonalMultiply)
  ->RangeMultiplier(2)
  ->Range(4, 256);

template<size_t size>
void StaticDenseDiagonalMultiply(benchmark::State& bench) {
  using MatrixS = Matrix<double, size, size>;
  using VectorS = Matrix<double, size, 1>;
  const MatrixS A = VectorS::Random().asDiagonal();
  const MatrixS B = VectorS::Random().asDiagonal();
  MatrixS C = MatrixS::Zero(size, size);

  for (auto _ : bench) {
    C = A * B;
    benchmark::DoNotOptimize(C);
  }

  bench.SetBytesProcessed(bench.iterations() * sizeof(double) * size * size);
}
BENCHMARK_TEMPLATE(StaticDenseDiagonalMultiply, 4);
BENCHMARK_TEMPLATE(StaticDenseDiagonalMultiply, 8);
BENCHMARK_TEMPLATE(StaticDenseDiagonalMultiply, 16);
BENCHMARK_TEMPLATE(StaticDenseDiagonalMultiply, 32);
BENCHMARK_TEMPLATE(StaticDenseDiagonalMultiply, 64);
BENCHMARK_TEMPLATE(StaticDenseDiagonalMultiply, 128);
BENCHMARK_TEMPLATE(StaticDenseDiagonalMultiply, 256);

template<size_t size>
void StaticDenseNoAliasDiagonalMultiply(benchmark::State& bench) {
  using MatrixS = Matrix<double, size, size>;
  using VectorS = Matrix<double, size, 1>;
  const MatrixS A = VectorS::Random().asDiagonal();
  const MatrixS B = VectorS::Random().asDiagonal();
  MatrixS C = MatrixS::Zero(size, size);

  for (auto _ : bench) {
    C.noalias() = A * B;
    benchmark::DoNotOptimize(C);
  }

  bench.SetBytesProcessed(bench.iterations() * sizeof(double) * size * size);
}
BENCHMARK_TEMPLATE(StaticDenseNoAliasDiagonalMultiply, 4);
BENCHMARK_TEMPLATE(StaticDenseNoAliasDiagonalMultiply, 8);
BENCHMARK_TEMPLATE(StaticDenseNoAliasDiagonalMultiply, 16);
BENCHMARK_TEMPLATE(StaticDenseNoAliasDiagonalMultiply, 32);
BENCHMARK_TEMPLATE(StaticDenseNoAliasDiagonalMultiply, 64);
BENCHMARK_TEMPLATE(StaticDenseNoAliasDiagonalMultiply, 128);
BENCHMARK_TEMPLATE(StaticDenseNoAliasDiagonalMultiply, 256);

void SparseMatrixMultiply(benchmark::State& bench) {
  const size_t size = bench.range(0);

  VectorXd diag_a = VectorXd::Random(size);
  VectorXd diag_b = VectorXd::Random(size);

  SparseMatrix<double> A(size, size);
  SparseMatrix<double> B(size, size);
  for (size_t i = 0; i < size; ++i) {
    A.insert(i, i) = diag_a(i);
    B.insert(i, i) = diag_b(i);
  }
  A.makeCompressed();
  B.makeCompressed();
  SparseMatrix<double> C(size, size);

  for (auto _ : bench) {
    C = A * B;
    benchmark::DoNotOptimize(C);
  }

  bench.SetBytesProcessed(bench.iterations() * sizeof(double) * size);
}
BENCHMARK(SparseMatrixMultiply)->RangeMultiplier(2)->Range(4, 256);