#include "agilib/utils/file_utils.hpp"

#include "agilib/utils/yaml.hpp"

namespace agi {

bool checkFile(const fs::path& directory, fs::path* const filename) {
  if (filename == nullptr) return false;

  if (filename->empty()) return false;

  if (fs::is_directory(*filename)) return false;

  if (filename->is_absolute()) {
    if (fs::exists(*filename)) return true;
  }

  if (fs::exists(directory / *filename)) {
    *filename = directory / *filename;
    return true;
  }

  return false;
}

fs::path getQuadFile(const Yaml& yaml, const fs::path& quad_file_yaml,
                     const fs::path& agi_param_directory) {
  fs::path quad_file = quad_file_yaml;
  std::vector<fs::path> possible_paths{agi_param_directory,
                                       agi_param_directory / "quads"};
  if (!quad_file.empty()) {
    if (quad_file.has_extension()) {
      if (quad_file.is_absolute()) {
        possible_paths.clear();
      }
    } else {
      if (quad_file.is_absolute()) {
        possible_paths.clear();
        possible_paths.push_back(quad_file);
        possible_paths.push_back(quad_file / "quads");
        quad_file.clear();
      } else {
        throw ParameterException(
          "If quadrotor file path is manually set, it must be an absolute "
          "path.\n"
          "Specified path: " +
          quad_file.string());
      }
    }
  }
  fs::path quad_file_from_yaml = [&yaml] {
    std::string quad_file_from_yaml;
    yaml["quadrotor"].getIfDefined(quad_file_from_yaml);
    return quad_file_from_yaml;
  }();
  
  fs::path quad_file_;
  if (!quad_file_from_yaml.empty()) {
    if (!quad_file.empty())
      throw ParameterException("Quadrotor file is set manually and in YAML!");

    if (!quad_file_from_yaml.has_extension())
      throw ParameterException("Quadrotor file is no filename!");

    quad_file_ = quad_file_from_yaml;

    if (quad_file_.is_absolute()) possible_paths.clear();
  }

  for (const fs::path& path : possible_paths) {
    if (checkFile(path, &quad_file_)) return quad_file_;
  }

  return quad_file_;
}

}  // namespace agi