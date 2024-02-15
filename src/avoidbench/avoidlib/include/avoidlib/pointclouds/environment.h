#pragma once
#include <assert.h>
#include <Eigen/Dense>
#include <cmath>
#include <cstring>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <thread>
#include <vector>

#include <Open3D/Geometry/KDTreeFlann.h>
#include <Open3D/Geometry/PointCloud.h>
#include <Open3D/IO/ClassIO/PointCloudIO.h>

#include <yaml-cpp/yaml.h>

namespace avoidlib
{
struct Bounding
{
    /* data */
    std::vector<double> bounding_origin_;
    std::vector<double> bounding_area_;
    double resolution_, resolution_inv_;
    int zWidth, zHeight, zLength;
};

struct float3 {
  float x, y, z;
};

class Environment {
public:
  Environment(const YAML::Node &cfg_path);
  ~Environment() {};
  Bounding bounding;
  double drone_r;
  double getTraversability();
  void readPointCloud(const std::string &filepath);
  Eigen::Vector3i pcl2Node(const Eigen::Vector3d &pos);
  Eigen::Vector3d Node2pcl(const Eigen::Vector3i &node);
  bool checkOccupied(const Eigen::Vector3d &pos, const double radius=0.0);
  bool checkOccupied(const Eigen::Vector3i &node);
  double getOccupiedDistance(const Eigen::Vector3d &pos,
                    const double radius, const double max_nn);
  bool checkIdxBounding(const Eigen::Vector3i &node);

private:
  open3d::geometry::KDTreeFlann kd_tree_;
  Eigen::MatrixXd points_;
  std::string file_name_;
  std::ofstream location_out;

  bool loadParameters(const YAML::Node &cfg);
  double getInitialCheckPointBias(const double x, const double dist, int* const layer);
  void calNeighborPoints(const int direct_num, const Eigen::Vector3d &query_point, double* const free_dist);
  bool checkInBounding(const Eigen::Vector3d& pt);


};
}