#pragma once

#include <unordered_map>

#include <glog/logging.h>

namespace rpg_common {

// For when the value is to be modified after emplacement - otherwise use
// CHECK(map.emplace(key, value).second);
// Note that due to umap re-allocation, the returned pointer is valid only
// temporarily.
template <typename KeyType, typename ValueType>
ValueType* emplaceChecked(
    const KeyType key, std::unordered_map<KeyType, ValueType>* destination)
{
  typedef std::unordered_map<KeyType, ValueType> Map;
  CHECK_NOTNULL(destination);
  std::pair<typename Map::iterator, bool> emplacement =
      destination->emplace(key, ValueType());
  CHECK(emplacement.second);
  return &emplacement.first->second;
}

}  // namespace rpg_common
namespace rpg = rpg_common;
