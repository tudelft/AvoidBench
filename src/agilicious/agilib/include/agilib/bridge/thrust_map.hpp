#pragma once

#include <agilib/utils/filesystem.hpp>

#include "agilib/math/types.hpp"

namespace agi {

struct ThrustMap {
  using NeighborCoordinates = std::pair<int, int>;

  ThrustMap() = default;
  ~ThrustMap() = default;

  bool load(const fs::path& file);

  bool valid();

  Scalar map(const Scalar thrust, const Scalar battery_voltage) const;
  inline NeighborCoordinates neighbor(const Scalar value, const Scalar lower,
                                      const Scalar upper, const int size,
                                      Scalar* const x) const;

  Matrix<> map_ = Matrix<>::Constant(1, 1, NAN);
  Scalar voltage_min_{NAN};
  Scalar voltage_max_{NAN};
  Scalar thrust_min_{NAN};
  Scalar thrust_max_{NAN};
};

}  // namespace agi