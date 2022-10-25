#pragma once

#include <unordered_map>
#include <unordered_set>

#include <glog/logging.h>

namespace rpg_common {

template <typename Type>
bool merge(const std::unordered_set<Type>& source,
           std::unordered_set<Type>* destination)
{
  CHECK_NOTNULL(destination)->insert(source.begin(), source. end());
  return true;
}

template <typename KeyType, typename ValueType>
bool merge(const std::unordered_map<KeyType, ValueType>& source,
           std::unordered_map<KeyType, ValueType>* destination)
{
  CHECK_NOTNULL(destination);
  for (const typename std::unordered_map<KeyType, ValueType>::value_type& item :
      source)
  {
    if (!destination->emplace(item).second)
    {
      if (!merge(item.second, &(*destination)[item.first]))
      {
        return false;
      }
    }
  }
  return true;
}

template <typename Type>
bool merge(const std::vector<Type>& source, Type* destination)
{
  CHECK_NOTNULL(destination);

  for (const Type& item : source)
  {
    if (!merge(item, destination))
    {
      return false;
    }
  }
  return true;
}

}  // namespace rpg_common
