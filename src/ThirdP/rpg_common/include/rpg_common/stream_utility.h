#pragma once

#include <iostream>
#include <map>
#include <set>
#include <utility>
#include <vector>

// With this, you can simply pass stl containers to ostreams.
// This not only prints its contents, but also handles indentation.
// For currently supported stl containers, see std::ostream << specializations
// below.

namespace std {

template <typename Type>
ostream& rpgCommonStreamUtilOut (
    ostream& stream, const size_t indent_level, const Type& item)
{
  return stream << item;
}

inline std::ostream& rpgCommonStreamUtilIndent(
    std::ostream& stream, const size_t indent_level)
{
  if (indent_level == 0u)
  {
    return stream;
  }
  else
  {
    return rpgCommonStreamUtilIndent(stream << "  ", indent_level - 1u);
  }
}

template <typename Type1, typename Type2>
ostream& rpgCommonStreamUtilOut (
    ostream& stream, const size_t indent_level,
    const map<Type1, Type2>& map)
{
  stream << "{" << endl;
  for(const typename std::map<Type1, Type2>::value_type& pair : map)
  {
    rpgCommonStreamUtilOut(rpgCommonStreamUtilOut(
        rpgCommonStreamUtilIndent(stream, indent_level + 1u),
        indent_level + 1u, pair.first) << ": ", indent_level + 1u, pair.second)
            << endl;
  }
  return rpgCommonStreamUtilIndent(stream, indent_level) << "}";
}

template <typename Type>
ostream& rpgCommonStreamUtilOut (
    ostream& stream, const size_t indent_level, const set<Type>& set)
{
  stream << "{" << endl;
  for(const Type& item : set)
  {
    rpgCommonStreamUtilOut(rpgCommonStreamUtilIndent(
        stream, indent_level + 1u), indent_level + 1u, item) <<
            endl;
  }
  return rpgCommonStreamUtilIndent(stream, indent_level) << "}";
}

template <typename Type>
ostream& rpgCommonStreamUtilOut (ostream& stream, const size_t indent_level,
                                 const vector<Type>& vector)
{
  stream << "{" << endl;
  for(const Type& item : vector)
  {
    rpgCommonStreamUtilOut(rpgCommonStreamUtilIndent(
        stream, indent_level + 1u), indent_level + 1u, item) << endl;
  }
  return rpgCommonStreamUtilIndent(stream, indent_level) << "}";
}

template <typename Type1, typename Type2>
ostream& operator<< (ostream& stream, const pair<Type1, Type2>& pair)
{
  return stream << "(" << pair.first << ", " << pair.second << ")";
}

template <typename Type1, typename Type2>
ostream& operator<< (ostream& stream, const map<Type1, Type2>& map)
{
  return rpgCommonStreamUtilOut(stream, 0u, map);
}

template <typename Type>
ostream& operator<< (ostream& stream, const set<Type>& set)
{
  return rpgCommonStreamUtilOut(stream, 0u, set);
}

template <typename Type>
ostream& operator<< (ostream& stream, const vector<Type>& vector)
{
  return rpgCommonStreamUtilOut(stream, 0u, vector);
}

}  // namespace std
