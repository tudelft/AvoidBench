
#define TINYPLY_IMPLEMENTATION
#include "tinyply.h"
#include "environment.h"
namespace avoidmetrics
{
  Environment::Environment(const YAML::Node &cfg_path)
  {
    if (!loadParameters(cfg_path)) {
      std::cout<<"load parameter failed 3"<<std::endl;
    }
    // readPointCloud();
  }

  void Environment::readPointCloud(const std::string &filepath) {
    std::shared_ptr<std::istream> file_stream;
    std::vector<uint8_t> byte_buffer;
    try {
      file_stream.reset(new std::ifstream(filepath, std::ios::binary));

      if (!file_stream || file_stream->fail())
        throw std::runtime_error("file_stream failed to open " + filepath);

      tinyply::PlyFile file;
      file.parse_header(*file_stream);

      std::cout << "\t[ply_header] Type: "
                << (file.is_binary_file() ? "binary" : "ascii") << std::endl;
      for (const auto &c : file.get_comments())
        std::cout << "\t[ply_header] Comment: " << c << std::endl;
      for (const auto &c : file.get_info())
        std::cout << "\t[ply_header] Info: " << c << std::endl;

      for (const auto &e : file.get_elements()) {
        std::cout << "\t[ply_header] element: " << e.name << " (" << e.size << ")"
                  << std::endl;
        for (const auto &p : e.properties) {
          std::cout << "\t[ply_header] \tproperty: " << p.name
                    << " (type=" << tinyply::PropertyTable[p.propertyType].str
                    << ")";
          if (p.isList)
            std::cout << " (list_type=" << tinyply::PropertyTable[p.listType].str
                      << ")";
          std::cout << std::endl;
        }
      }

      // Because most people have their own mesh types, tinyply treats parsed data
      // as structured/typed byte buffers. See examples below on how to marry your
      // own application-specific data structures with this one.
      std::shared_ptr<tinyply::PlyData> vertices, normals, colors, texcoords, faces,
        tripstrip;

      // The header information can be used to programmatically extract properties
      // on elements known to exist in the header prior to reading the data. For
      // brevity of this sample, properties like vertex position are hard-coded:
      try {
        vertices =
          file.request_properties_from_element("vertex", {"x", "y", "z"});
      } catch (const std::exception &e) {
        std::cerr << "tinyply exception: " << e.what() << std::endl;
      }

      file.read(*file_stream);

      if (vertices)
        std::cout << "\tRead " << vertices->count << " total vertices "
                  << std::endl;


      const size_t numVerticesBytes = vertices->buffer.size_bytes();


      std::vector<float3> verts(vertices->count);
      std::memcpy(verts.data(), vertices->buffer.get(), numVerticesBytes);

      int idx = 0;
      for (const auto &point_tinyply : verts) {
        if (idx == 0) {
          points_ = Eigen::Vector3d(static_cast<double>(point_tinyply.x),
                                    static_cast<double>(point_tinyply.y),
                                    static_cast<double>(point_tinyply.z));
        } else {
          points_.conservativeResize(points_.rows(), points_.cols() + 1);
          points_.col(points_.cols() - 1) =
            Eigen::Vector3d(static_cast<double>(point_tinyply.x),
                            static_cast<double>(point_tinyply.y),
                            static_cast<double>(point_tinyply.z));
        }
        idx += 1;
      }

      kd_tree_.SetMatrixData(points_);

    } catch (const std::exception &e) {
      std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
    }
  }

  bool Environment::loadParameters(const YAML::Node &cfg)
  {
    if (cfg["unity"])
    {
      bounding.bounding_area_ = cfg["unity"]["range"].as<std::vector<double>>();
      bounding.bounding_origin_ = cfg["unity"]["origin"].as<std::vector<double>>();
      bounding.resolution_ = cfg["unity"]["res"].as<double>();
      bounding.resolution_inv_ = 1 / bounding.resolution_;
      bounding.zWidth = std::floor(bounding.bounding_area_[0] / bounding.resolution_) + 1;
      bounding.zLength = std::floor(bounding.bounding_area_[1] / bounding.resolution_) + 1;
      bounding.zHeight = std::floor(bounding.bounding_area_[2] / bounding.resolution_) + 1;
      double width = cfg["drone"]["width"].as<double>();
      double length = cfg["drone"]["length"].as<double>();
      drone_r = width / 2.0;
      return true;
    }
    return false;
  }

  double Environment::getInitialCheckPointBias(const double x, const double dist, int* const layer)
  {
    double bias;
    if (std::floor(x / dist)>1)
    {
      bias = (x - std::floor(x / dist) * dist + dist)/2.0;
      *layer = std::floor(x / dist) - 1;
    }
    else {
      bias = x/2.0;
      *layer = 1;
    }
    return bias;
  }

  bool Environment::checkInBounding(const Eigen::Vector3d& pt)
  {
    if(abs(pt.x()-bounding.bounding_origin_[0])>=bounding.bounding_area_[0]/2.0) return false;
    if(abs(pt.y()-bounding.bounding_origin_[1])>=bounding.bounding_area_[1]/2.0) return false;
    if(abs(pt.z()-bounding.bounding_origin_[2])>=bounding.bounding_area_[2]/2.0) return false;
    return true;
  }

  double Environment::getTraversability()
  {
    double check_dist = 1.0;
    Eigen::Vector3d initial_checked_point, vert_point;
    vert_point.x() = bounding.bounding_origin_[0] - bounding.bounding_area_[0]/2.0;
    vert_point.y() = bounding.bounding_origin_[1] - bounding.bounding_area_[1]/2.0;
    vert_point.z() = bounding.bounding_origin_[2] - bounding.bounding_area_[2]/2.0;
    double bias_x, bias_y, bias_z;
    int layer_x, layer_y, layer_z;
    bias_x = getInitialCheckPointBias(bounding.bounding_area_[0], check_dist, &layer_x);
    bias_y = getInitialCheckPointBias(bounding.bounding_area_[1], check_dist, &layer_y);
    bias_z = getInitialCheckPointBias(bounding.bounding_area_[2], check_dist, &layer_z);
    initial_checked_point = vert_point + Eigen::Vector3d(bias_x, bias_y, bias_z);
    double traversability = 0;

    for(int i=0; i<layer_x; i++)
    {
      for(int j=0; j<layer_y; j++)
      {
        for(int k=0; k<layer_z; k++)
        {
          double average_free_distance;
          Eigen::Vector3d checked_point = initial_checked_point + 
                                          Eigen::Vector3d(i*check_dist, j*check_dist, k*check_dist);
          calNeighborPoints(8, checked_point, &average_free_distance);
          // std::cout<<"checked_point: "<<checked_point.transpose()<<std::endl;
          traversability = traversability + average_free_distance;
        }
      }
    }
    traversability = traversability / (layer_x*layer_y*layer_z);
    return traversability;
  }

  void Environment::calNeighborPoints(const int direct_num, const Eigen::Vector3d &query_point,
                                      double* const free_dist)
  {
    location_out.open(getenv("AVOIDBENCH_PATH") + std::string("/avoidmetrics/data/check_points_out.txt"), 
                          std::ios::out | std::ios::app);
    std::vector<int> indices;
    std::vector<double> distances_squared;
    kd_tree_.SearchRadius(query_point, bounding.resolution_, indices, distances_squared);
    if (indices.size() != 0) {
      *free_dist = 0;
      // for(int i=0; i<direct_num; i++)
      // {
      //   std::string ss = std::to_string(0) + "\n";
      //   location_out << ss;
      // }
      // location_out.close();
      return;
    }
    double aver_dist=0;
    double step_angle = 2*M_PI/direct_num;

    for(int i=0; i<direct_num; i++)
    {
      double direction = i*step_angle;
      Eigen::Vector3d ray_point = query_point+Eigen::Vector3d(bounding.resolution_*sin(direction),
                                              bounding.resolution_*cos(direction), 0);
      kd_tree_.SearchRadius(ray_point, bounding.resolution_, indices, distances_squared);
      while(indices.size()==0)
      {
        ray_point = ray_point+Eigen::Vector3d(bounding.resolution_*sin(direction),
                                              bounding.resolution_*cos(direction), 0);

        kd_tree_.SearchRadius(ray_point, bounding.resolution_, indices, distances_squared);
        if(!checkInBounding(ray_point)) 
        {
          ray_point = ray_point - Eigen::Vector3d(bounding.resolution_*sin(direction),
                                              bounding.resolution_*cos(direction), 0);
          break;        
        }
      }
      double dist = (ray_point - query_point).norm();
      aver_dist = aver_dist + dist;
      // std::string ss = std::to_string(dist) + "\n";
      // location_out << ss;
    }
    // location_out.close();
    // exit(1);
    aver_dist = aver_dist / direct_num;
    *free_dist = aver_dist;
  }

  Eigen::Vector3i Environment::pcl2Node(const Eigen::Vector3d &pos)
  {
    Eigen::Vector3i node;
    double x = bounding.resolution_inv_ * (pos.x() + bounding.bounding_area_[0]/2.0 - bounding.bounding_origin_[0]);
    double xi = x - std::floor(x);
    if(xi > 0.5) x = std::ceil(x);

    double y = bounding.resolution_inv_ * (pos.y() + bounding.bounding_area_[1]/2.0 - bounding.bounding_origin_[1]);
    double yi = y - std::floor(y);
    if(yi > 0.5) y = std::ceil(y);

    double z = bounding.resolution_inv_ * (pos.z() + bounding.bounding_area_[2]/2.0 - bounding.bounding_origin_[2]);
    double zi = z - std::floor(z);
    if(zi > 0.5) z = std::ceil(z);

    node.x() = x;
    node.y() = y;
    node.z() = z;
    node.x() = std::max(std::min(node.x(), bounding.zWidth), 0);
    node.y() = std::max(std::min(node.y(), bounding.zLength), 0);
    node.z() = std::max(std::min(node.z(), bounding.zHeight), 0);
    return node;
  }

  Eigen::Vector3d Environment::Node2pcl(const Eigen::Vector3i &node)
  {
    Eigen::Vector3d pos;
    pos.x() = bounding.resolution_ * node.x() + bounding.bounding_origin_[0] - bounding.bounding_area_[0]/2.0;
    pos.y() = bounding.resolution_ * node.y() + bounding.bounding_origin_[1] - bounding.bounding_area_[1]/2.0;
    pos.z() = bounding.resolution_ * node.z() + bounding.bounding_origin_[2] - bounding.bounding_area_[2]/2.0;
    return pos;
  }

  bool Environment::checkOccupied(const Eigen::Vector3d &pos)
  {
    std::vector<int> indices;
    std::vector<double> distances_squared;
    kd_tree_.SearchRadius(pos, drone_r, indices, distances_squared);
    if (indices.size() != 0) {
      return true;
    }
    else return false;
  }

  bool Environment::checkOccupied(const Eigen::Vector3i &node)
  {
    Eigen::Vector3d pos = Node2pcl(node);
    std::vector<int> indices;
    std::vector<double> distances_squared;
    kd_tree_.SearchRadius(pos, drone_r, indices, distances_squared);
    if (indices.size() != 0) {
      return true;
    }
    else return false;
  }

  bool Environment::checkIdxBounding(const Eigen::Vector3i &node)
  {
    if(node.x()>=0 && node.x()<bounding.zWidth && node.y()>=0 && node.y()<bounding.zLength &&
      node.z()>=0 && node.z()<bounding.zHeight)
      return true;
    else return false;
  }

}