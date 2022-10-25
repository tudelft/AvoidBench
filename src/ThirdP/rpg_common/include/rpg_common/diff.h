#pragma once

#include <vector>

#include <glog/logging.h>

namespace rpg_common
{
namespace diff {

// Assumes front items are kept the same in general.
template <typename Type>
struct StlVectorDiff
{
  size_t num_same_front_items;
  std::vector<Type> new_items_after_that;
  std::vector<Type> old_items_after_that;

  StlVectorDiff(const std::vector<Type>& original,
                const std::vector<Type>& new_vector)
  {
    num_same_front_items = 0u;
    const size_t shorter_size = std::min(original.size(), new_vector.size());
    for (size_t i = 0u; i < shorter_size; ++i)
    {
      if (new_vector[i] == original[i])
      {
        num_same_front_items = i + 1u;
      }
      else
      {
        break;
      }
    }
    if (num_same_front_items < new_vector.size())
    {
      new_items_after_that.insert(
          new_items_after_that.end(), new_vector.begin() + num_same_front_items,
          new_vector.end());
    }
    if (num_same_front_items < original.size())
    {
      old_items_after_that.insert(
          old_items_after_that.end(), original.begin() + num_same_front_items,
          original.end());
    }
  }

  bool isAddOnly() const
  {
    return old_items_after_that.empty();
  }

  bool hasChange() const
  {
    return !(new_items_after_that.empty() && old_items_after_that.empty());
  }

  void patch(std::vector<Type>* destination) const
  {
    CHECK_NOTNULL(destination)->resize(num_same_front_items);
    destination->insert(destination->end(), new_items_after_that.begin(),
                        new_items_after_that.end());
  }
};

}  // namespace diff

}  // namespace rpg_common
namespace rpg = rpg_common;
