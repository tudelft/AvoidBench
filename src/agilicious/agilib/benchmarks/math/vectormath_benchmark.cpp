#include <Eigen/Eigen>

#include "benchmark/benchmark.h"

using namespace Eigen;


void VectorDynamicMultiply(benchmark::State& bench) {
  const size_t size = bench.range(0);

  const VectorXd x = VectorXd::Random(size);
  const VectorXd y = VectorXd::Random(size);

  for (auto _ : bench) {
    const double dp = x.transpose() * y;
    benchmark::DoNotOptimize(dp);
  }

  bench.SetBytesProcessed(bench.iterations() * sizeof(double) * size * 2);
}
BENCHMARK(VectorDynamicMultiply)->RangeMultiplier(2)->Range(4, 256);

template<size_t size>
void VectorStaticMultiply(benchmark::State& bench) {
  using VectorS = Matrix<double, size, 1>;

  const VectorS x = VectorS::Random(size);
  const VectorS y = VectorS::Random(size);

  for (auto _ : bench) {
    const double dp = x.transpose() * y;
    benchmark::DoNotOptimize(dp);
  }

  bench.SetBytesProcessed(bench.iterations() * sizeof(double) * size * 2);
}
BENCHMARK_TEMPLATE(VectorStaticMultiply, 4);
BENCHMARK_TEMPLATE(VectorStaticMultiply, 8);
BENCHMARK_TEMPLATE(VectorStaticMultiply, 16);
BENCHMARK_TEMPLATE(VectorStaticMultiply, 32);
BENCHMARK_TEMPLATE(VectorStaticMultiply, 64);
BENCHMARK_TEMPLATE(VectorStaticMultiply, 128);
BENCHMARK_TEMPLATE(VectorStaticMultiply, 256);
