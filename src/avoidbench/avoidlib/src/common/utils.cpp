#include "avoidlib/common/utils.hpp"

namespace avoidlib {

bool file_exists(const std::filesystem::path& p) {
  return (std::filesystem::exists(p));
}

}  // namespace flightlib
