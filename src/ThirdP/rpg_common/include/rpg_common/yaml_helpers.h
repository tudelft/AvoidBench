/*
 * yaml_helpers.h
 *
 *  Created on: Aug 26, 2016
 *      Author: Michael Gassner
 *
 *  Extends the yaml convert() function to make it possible to load and write some common
 *  types (e.g. constant size Eigen::Matrix) from a yaml file.
 *
 *  Requirements:
 *  add to your package.xml:
 *  <depend>rpg_common<depend>
 *
 *  add to your CMakeLists.txt:
 *  cs_add_executable(your_executable  some_files.cpp ...)
 *  target_link_libraries(your_executable yaml-cpp)
 *
 *  add to your .cpp/.h file:
 *  #include "rpg_common/yaml_helpers.h"
 *
 *  Usage example to load a 2x3 matrix | 1.0 2.0 3.0 |
 *                                     | 4.0 5.0 6.0 |
 *
 *  YAML::Node yaml_root_node;
 *  try{
 *    yaml_root_node = YAML::Load("my_matrix: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]");
 *    // or use:
 *    // yaml_root_node = YAML::LoadFile("/path/to/file.yaml");
 *    // where file.yaml:
 *    // my_matrix: [1.0, 2.0, 3.0,
 *    //             4.0, 5.0, 6.0]
 *
 *  } catch (const std::exception& ex) {
 *  std::cout << "Failed to load yamlfile: \n"
 *     << ex.what() << std::endl;
 *     // [...Do failure recovery....]
 *  }
 *
 *  if(yaml_root_node["my_matrix"])
 *  {
 *     Eigen::Matrix<double, 2, 3> my_matrix = yaml_root_node["my_matrix"].as<Eigen::Matrix<double, 2, 3>>();
 *     std::cout << my_matrix << std::endl;
 *     std::vector<double> read_as_vector(6); // size of std::vector must be allocated otherwise load will fail
 *     read_as_vector = yaml_root_node["my_matrix"].as<std::vector<double>>()
 *     std::vector<double> read_as_vector_at_construction_time = yaml_root_node["my_matrix"].as<std::vector<double>>();
 *  }
 *  else
 *  {
 *    // "my_matrix" was not defined in .yaml file.
 *  }
 *
 *
 */

#pragma once

#include "yaml-cpp/yaml.h"
#include <Eigen/Core>
#include <glog/logging.h>


namespace YAML {

//************* Convert Eigen::Matrix<_type, _rows, _cols, _options> from and to yaml ********************
template<class _type, int _rows, int _cols, int _options>
struct convert< Eigen::Matrix<_type, _rows, _cols, _options> > {
  static Node encode(const Eigen::Matrix<_type, _rows, _cols, _options>& rhs) {
    static_assert(_rows >= 0 && _cols >= 0, "Dynamically sized matrices not supported yet");

    Node node;
    if(_options == Eigen::ColMajor)
    {
      for(size_t row = 0; row < _rows; ++row)
      {
        for(size_t col = 0; col < _cols; ++col)
        {
          node.push_back(rhs(row, col));
        }
      }
    }
    else if(_options == Eigen::RowMajor)
    {
      for(size_t col = 0; col < _cols; ++col)
      {
        for(size_t row = 0; row < _rows; ++row)
        {
          node.push_back(rhs(row, col));
        }
      }
    }
    else
    {
      for(size_t i = 0; i < rhs.size(); ++i)
      {
        node.push_back(rhs(i));
      }
    }

    return node;
  }

  static bool decode(const Node& node, Eigen::Matrix<_type, _rows, _cols, _options>& rhs) {
    static_assert(_rows >= 0 && _cols >= 0, "Dynamically sized matrices not supported yet");
    CHECK_EQ(node.size(), static_cast<size_t>(rhs.size()));
    CHECK(node.IsSequence());

    if(!node.IsSequence() || node.size() != static_cast<size_t>(rhs.size())) {
      return false;
    }

    if(_options == Eigen::ColMajor)
    {
      for(size_t row = 0, i = 0; row < _rows; ++row)
      {
        for(size_t col = 0; col < _cols; ++col, ++i)
        {
          rhs(row, col) = node[i].as<_type>();
        }
      }
    }
    else if(_options == Eigen::RowMajor)
    {
      for(size_t col = 0, i = 0; col < _cols; ++col)
      {
        for(size_t row = 0; row < _rows; ++row, ++i)
        {
          rhs(row, col) = node[i].as<_type>();
        }
      }
    }
    else
    {
      LOG(FATAL) << "Matrix type not known";
      return false;
    }

    return true;
  }
};

} // namespace YAML

namespace yaml_helpers
{

//************* checked load from a node ********************
template<class _type>
bool load_from_yaml_node(const YAML::Node& root_node, const std::string& node_name, _type& out)
{
  if(root_node[node_name])
  {
    out = root_node[node_name].as<_type>();
  }
  else
  {
    return false;
  }
  return true;
}

} // namespace yaml_helpers

