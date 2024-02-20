#include "agilib/utils/timer.hpp"

#include <gtest/gtest.h>

#include "agilib/math/types.hpp"

using namespace agi;

static constexpr Scalar tol = 1e-3;


/// Example of Timer as unit test
TEST(Timer, SimpleTiming) {
  static constexpr int N = 100;
  static constexpr int dt = 10000;  // us

  Timer timer("Example", "UnitTest");

  for (int i = 0; i < N; ++i) {
    timer.tic();
    usleep(dt);
    timer.toc();
  }

  std::cout << timer;

  EXPECT_EQ(timer.count(), N);
  EXPECT_NEAR(timer.mean(), dt * 1e-6, tol);
}

TEST(Timer, AdvancedTiming) {
  static constexpr int dt_min = 10000;
  static constexpr int dt_max = 20000;
  static constexpr int dt_step = 1000;

  Timer timer;

  for (int dt = dt_min; dt <= dt_max; dt += dt_step) {
    timer.tic();
    usleep(dt);
    timer.toc();
  }

  EXPECT_NEAR(timer.min(), 1e-6 * dt_min, tol);
  EXPECT_NEAR(timer.mean(), 1e-6 * (dt_min + dt_max) / 2, tol);
  EXPECT_NEAR(timer.max(), 1e-6 * dt_max, tol);
  EXPECT_NEAR(timer.std(), 1e-6 * (dt_max - dt_min) / sqrt(12.0), tol);
  EXPECT_EQ(timer.count(), (int)(1 + (dt_max - dt_min) / dt_step));
}

/// Example of Scopedtimer as unit test.
TEST(Timer, ScopedTimer) {
  static constexpr int dt = 1e5;
  static constexpr Scalar tol = 1e-3;

  Timer timer;
  {                    // As an example, a scope to put our timer in...
    ScopedTimer timy;  // ...and the timer it self. This is all you need to do.

    timer = timy;  // This thing you dont need. It's only for unit testing.

    usleep(dt);  // This would be the code to time.
  }  // Here the scope ends, the ScopedTimer gets destroyed and prints

  // Everything below is just unit testing and not part of the example.
  timer.toc();
  EXPECT_NEAR(timer.mean(), 1e-6 * dt, tol);
}