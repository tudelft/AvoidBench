#pragma once

#include "agilib/base/parameter_base.hpp"
#include "agilib/utils/filesystem.hpp"


namespace agi {

bool checkFile(const fs::path& directory, fs::path* const filename);

fs::path getQuadFile(const Yaml& yaml, const fs::path& quad_file,
                     const fs::path& agi_param_directory);

}  // namespace agi
