#pragma once

#include <algorithm>
#include <vector>

#include <glog/logging.h>

namespace rpg_common {

// Returns indices of the k lowest elements. Inspired by
// http://stackoverflow.com/questions/10580982/c-sort-keeping-track-of-indices#10581051
// except that nth_element is faster than sort (result is not ordered).
template <typename Type>
inline std::vector<size_t> k_lowest_indices(
    const std::vector<Type>& input, const size_t k)
{
  CHECK_GT(input.size(), k);
  std::vector<size_t> indices(input.size());
  std::iota(indices.begin(), indices.end(), 0u);

  std::nth_element(indices.begin(), indices.begin() + k - 1, indices.end(),
      [&](const size_t a, const size_t b) { return input[a] < input[b]; });
  indices.resize(k);
  return indices;
}

}  // namespace rpg_common
namespace rpg = rpg_common;
