#pragma once

namespace rpg_common {

// Merges source into destination for any data structure; returns false if
// the merge fails.
template <typename Type>
bool merge(const Type& source, Type* destination);

// Merges all items from source into destination.
template <typename Type>
bool merge(const std::vector<Type>& source, Type* destination);

}  // namespace rpg_common
namespace rpg = rpg_common;

#include "./merge-inl.h"
