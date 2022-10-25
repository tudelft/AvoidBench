#pragma once
// Copied from ASL's multiagent_mapping_basic

#include <unordered_map>

#include <glog/logging.h>

// Returns a const reference to the value for the specified key in an std::unordered_map.
// Fails hard if the keys does not exist.
namespace rpg_common
{

template <typename KeyType, typename ValueType, template <typename> class Hash,
          template <typename> class Comparator,
          template <typename> class Allocator>
inline const ValueType& getChecked(
    const std::unordered_map<
        KeyType, ValueType, Hash<KeyType>, Comparator<KeyType>,
        Allocator<std::pair<const KeyType, ValueType> > >& map,
    const KeyType& key)
{
  typename std::unordered_map<
      KeyType, ValueType, std::hash<KeyType>, std::equal_to<KeyType>,
      Allocator<std::pair<const KeyType, ValueType> > >::const_iterator it =
      map.find(key);
  CHECK(it != map.end());
  return it->second;
}

// Returns a non-const reference to the value for the specified key in an
// std::unordered_map. Fails hard if the keys does not exist.
template <typename KeyType, typename ValueType, template <typename> class Hash,
          template <typename> class Comparator,
          template <typename> class Allocator>
inline ValueType& getChecked(
    std::unordered_map<KeyType, ValueType, Hash<KeyType>, Comparator<KeyType>,
                       Allocator<std::pair<const KeyType, ValueType> > >&
        map,  // NOLINT
    const KeyType& key)
{
  typename std::unordered_map<
      KeyType, ValueType, std::hash<KeyType>, std::equal_to<KeyType>,
      Allocator<std::pair<const KeyType, ValueType> > >::iterator it =
      map.find(key);
  CHECK(it != map.end());
  return it->second;
}

}  // namespace rpg_common
namespace rpg = rpg_common;
