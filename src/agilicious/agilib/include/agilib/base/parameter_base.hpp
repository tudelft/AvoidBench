#pragma once

#include <agilib/utils/filesystem.hpp>
#include <exception>
#include <sstream>

#include "agilib/math/types.hpp"
#include "agilib/utils/filesystem.hpp"
#include "agilib/utils/yaml.hpp"

namespace agi {


struct ParameterException : public std::exception {
  ParameterException() = default;
  ParameterException(const std::string& msg)
    : msg(std::string("Agilib Parameter Exception: ") + msg) {}
  const char* what() const throw() { return msg.c_str(); }

  const std::string msg{"Agilib Parameter Exception"};
};

struct ParameterBase {
  virtual ~ParameterBase() = default;
  virtual bool load(const fs::path& filename);
  virtual bool load(const Yaml& yaml);

  virtual bool valid() const;
};

}  // namespace agi
