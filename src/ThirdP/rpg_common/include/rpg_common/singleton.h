#pragma once

namespace rpg_common {

// How to singleton:
// 1. Derive Class from rpg_common::Singleton<Class>.
// 2. Add RPG_COMMON_SINGLETON_PRIVATE(Class); to private.

#define RPG_COMMON_SINGLETON_PRIVATE(Class) \
  Class() = default; \
  Class(const Class&) = delete; \
  void operator =(const Class&) = delete; \
  friend rpg_common::Singleton<Class>

template <typename Derived>
class Singleton
{
public:
  static Derived& instance()
  {
    static Derived instance;
    return instance;
  }
};

}  // namespace rpg_common
namespace rpg = rpg_common;
