
#include "agilib/utils/low_pass_filter.hpp"

#include <gtest/gtest.h>

using namespace agi;

static constexpr int N = 128;
static constexpr Scalar cutoff_frequency = 1.0;
static constexpr Scalar sample_frequency = 100.0;

TEST(LowPassFilter, DefaultConstructorIsInvalid) {
  LowPassFilter<N> filter;
  EXPECT_FALSE(filter.valid());
}


TEST(LowPassFilter, Constructor) {
  LowPassFilter<N> filter(cutoff_frequency, sample_frequency, 0.0);
  EXPECT_TRUE(filter.valid());
}

TEST(LowPassFilter, OutputInRange) {
  for (int i = 0; i < N; ++i) {
    const Vector<N> x0 = 0.3 * Vector<N>::Random();
    LowPassFilter<N> filter(Vector<N>::Constant(cutoff_frequency),
                            Vector<N>::Constant(sample_frequency), x0);

    EXPECT_TRUE(filter().isApprox(x0));
    EXPECT_TRUE((filter().array() < 1.0).all());
    EXPECT_TRUE((filter().array() > -1.0).all());

    const Vector<N> x_sample = Vector<N>::Ones();
    for (int j = 0; j < 10; ++j) {
      const Vector<N> x_prev = filter();
      filter.add(x_sample);
      const Vector<N> x_new = filter();
      EXPECT_TRUE(((x_new - x_prev).array() > 0.0).all());
      EXPECT_TRUE((x_new.array() < 1.0).all());
      EXPECT_TRUE((filter.derivative().array() > 0.0).all());
    }
  }
}