//
// This bridge message types was originally from FlightGoggles.
// We made several changes on top of it.
//
#pragma once

// std
#include <string>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// avoidlib
#include "avoidlib/common/types.hpp"
#include "avoidlib/json/json.hpp"

using json = nlohmann::json;

namespace avoidlib {

enum UnityScene {
  INDUSTRIAL = 0,
  WAREHOUSE = 1,
  GARAGE = 2,
  TUNELS = 4,
  NATUREFOREST = 3,
  WAREHOUSE_KIT = 5,
  // total number of environment
  SceneNum = 6
};

// Unity Camera, should not be used alone.
// has to be attached on a vehicle.
struct Camera_t {
  std::string ID;
  // frame Metadata
  int channels{3};
  int width{1024};
  int height{768};
  float fov{70.0f};
  // Clip Planes for the different layers
  std::vector<float> near_clip_plane{0.01, 0.01, 0.01, 0.01};
  std::vector<float> far_clip_plane{1000.0, 100.0, 1000.0, 1000.0};
  float depth_scale{0.20};  // 0.xx corresponds to xx cm resolution
  // metadata
  bool is_depth{false};
  int output_index{0};
  //
  std::vector<bool> enabled_layers;
  // Transformation matrix from camera to vehicle body 4 x 4
  // use 1-D vector for json convention
  std::vector<float> T_BC{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

struct Lidar_t {
  std::string ID;
  int num_beams{10};
  float max_distance{10.0};
  float start_scan_angle{-M_PI / 2};
  float end_scan_angle{M_PI / 2};
  // Transformation matrix from lidar to vehicle body 4 x 4
  // use 1-D vector for json convention
  std::vector<float> T_BS{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

// Unity Vechicle, e.g., quadrotor
struct Vehicle_t {
  std::string ID;
  // unity coordinate system left-handed, y up
  std::vector<float> position{0.0, 0.0, 0.0};
  // unity quaternion (x, y, z, w)
  std::vector<float> rotation{0.0, 0.0, 0.0, 1.0};
  std::vector<float> size{1.0, 1.0, 1.0};  // scale
  // sensors attached on the vehicle
  std::vector<Camera_t> cameras;
  std::vector<Lidar_t> lidars;
  // collision check
  bool has_collision_check = true;
};

// Unity object, e.g., gate, light, etc...
struct Object_t {
  std::string ID;
  std::string prefab_ID;
  // unity coordinate system left hand, y up
  std::vector<float> position{0.0, 0.0, 0.0};
  // unity quaternion (x, y, z, w)
  std::vector<float> rotation{0.0, 0.0, 0.0, 1.0};
  std::vector<float> size{1.0, 1.0, 1.0};  // scale
  float opacity;
};

struct Object_Message_t {
  std::string name;
  std::vector<float> bounding_area{0.0, 0.0};
  std::vector<float> bounding_origin{0.0, 0.0};
  float radius;
  float opacity;
  int seed;
  std::vector<float> scale_min{0.5, 0.5, 1.3};
  std::vector<float> scale_max{1.0, 1.0, 1.8};
};

struct Tree_Message_t {
  std::string name;
  std::vector<float> bounding_area{0.0, 0.0};
  std::vector<float> bounding_origin{0.0, 0.0};
  float radius;
  int seed;
  std::vector<float> scale_min{0.5, 0.5, 1.3};
  std::vector<float> scale_max{1.0, 1.0, 1.8};
};

struct SettingsMessage_t {
  // scene/render settings
  size_t scene_id = UnityScene::WAREHOUSE;

  //
  std::vector<Vehicle_t> vehicles;
  std::vector<Object_t> objects;
  Object_Message_t object_message;
  Tree_Message_t tree_message;
};

struct PubMessage_t {
  FrameID frame_id{0};
  std::vector<Vehicle_t> vehicles;
  std::vector<Object_t> objects;
  Object_Message_t object_message;
  Tree_Message_t tree_message;
  bool spawn_new_obj{false};
};

//
struct Sub_Vehicle_t {
  bool collision;
  std::vector<float> lidar_ranges;
};

struct SubMessage_t {
  //
  FrameID frame_id{0};
  //
  std::vector<Sub_Vehicle_t> sub_vehicles;
  bool scene_change_feedback;
};

struct sub_pointcloud_state
{
  bool get_pc_msg{false};
  bool save_pc_success{false};
};

struct sub_collision_state
{
  bool get_msg{false};
  bool if_collision{false};
  std::vector<float> new_point{0.0, 0.0, 0.0};
};

struct PointCloudMessage_t {
  // define point cloud box range [x, y, z] / meter
  std::vector<float> range{20.0, 20.0, 20.0};
  std::vector<float> origin{0.0, 0.0, 0.0};
  float resolution{0.15};
  std::string path{getenv("AVOIDBENCH_PATH") + std::string("/avoidmetrics/point_clouds_data/")};
  std::string file_name{"default"};
};

struct CollisionCheckMessage_t {
  std::vector<float> checked_point{0.0, 0.0, 0.0};
  float drone_width{0.8};
  bool if_start_point{false};
};

/*********************
 * JSON constructors *
 *********************/
// Camera_t
inline void to_json(json &j, const Camera_t &o) {
  j = json{{"ID", o.ID},
           {"channels", o.channels},
           {"width", o.width},
           {"height", o.height},
           {"fov", o.fov},
           {"nearClipPlane", o.near_clip_plane},
           {"farClipPlane", o.far_clip_plane},
           {"T_BC", o.T_BC},
           {"isDepth", o.is_depth},
           {"enabledLayers", o.enabled_layers},
           {"depthScale", o.depth_scale},
           {"outputIndex", o.output_index}};
}

// Lidar
inline void to_json(json &j, const Lidar_t &o) {
  j = json{{"ID", o.ID},
           {"num_beams", o.num_beams},
           {"max_distance", o.max_distance},
           {"start_angle", o.start_scan_angle},
           {"end_angle", o.end_scan_angle},
           {"T_BS", o.T_BS}};
}
// Vehicle_t
inline void to_json(json &j, const Vehicle_t &o) {
  j = json{{"ID", o.ID},
           {"position", o.position},
           {"rotation", o.rotation},
           {"size", o.size},
           {"cameras", o.cameras},
           {"lidars", o.lidars},
           {"hasCollisionCheck", o.has_collision_check}};
}

// Object_t
inline void to_json(json &j, const Object_t &o) {
  j = json{{"ID", o.ID},
           {"prefabID", o.prefab_ID},
           {"position", o.position},
           {"rotation", o.rotation},
           {"size", o.size},
           {"opacity", o.opacity}};
}

inline void to_json(json &j, const Object_Message_t &o) {
  j = json{{"name", o.name},
           {"bounding_area", o.bounding_area},
           {"bounding_origin", o.bounding_origin},
           {"radius", o.radius},
           {"opacity", o.opacity},
           {"seed", o.seed},
           {"scale_min", o.scale_min},
           {"scale_max", o.scale_max}};
}

inline void to_json(json &j, const Tree_Message_t &o) {
  j = json{{"name", o.name},
           {"bounding_area", o.bounding_area},
           {"bounding_origin", o.bounding_origin},
           {"radius", o.radius},
           {"seed", o.seed},
           {"scale_min", o.scale_min},
           {"scale_max", o.scale_max}};
}

// Setting messages, pub to unity
inline void to_json(json &j, const SettingsMessage_t &o) {
  j = json{
    {"scene_id", o.scene_id}, {"vehicles", o.vehicles}, {"objects", o.objects}, {"object_message", o.object_message}, {"tree_message", o.tree_message}};
}

// Publish messages to unity
inline void to_json(json &j, const PubMessage_t &o) {
  j = json{
    {"frame_id", o.frame_id}, {"vehicles", o.vehicles}, {"objects", o.objects}, {"object_message", o.object_message}, {"tree_message", o.tree_message}, 
    {"spawn_new_obj", o.spawn_new_obj}};
}

// Publish messages to unity
inline void from_json(const json &j, Sub_Vehicle_t &o) {
  o.collision = j.at("collision").get<bool>();
  o.lidar_ranges = j.at("lidar_ranges").get<std::vector<float>>();
}

// json to our sub message data type
// note: pub_vechicles is defined in Unity which corresponding
// to our sub_vehicles in ROS.
inline void from_json(const json &j, SubMessage_t &o) {
  o.frame_id = j.at("frame_id").get<uint64_t>();
  o.sub_vehicles = j.at("pub_vehicles").get<std::vector<Sub_Vehicle_t>>();
  o.scene_change_feedback = j.at("scene_change_feedback").get<bool>();
}

inline void from_json(const json &j, sub_pointcloud_state &o) {
  o.save_pc_success = j.at("save_pc_success").get<bool>();
  o.get_pc_msg = j.at("get_pc_msg").get<bool>();
}

inline void from_json(const json &j, sub_collision_state &o) {
  o.get_msg = j.at("get_msg").get<bool>();
  o.if_collision = j.at("if_collision").get<bool>();
  o.new_point = j.at("new_point").get<std::vector<float>>();
}

inline void to_json(json &j, const PointCloudMessage_t &o) {
  j = json{{"range", o.range},
           {"origin", o.origin},
           {"resolution", o.resolution},
           {"path", o.path},
           {"file_name", o.file_name}};
}

inline void to_json(json &j, const CollisionCheckMessage_t &o) {
  j = json{{"checked_point", o.checked_point},
            {"drone_width", o.drone_width},
            {"if_start_point", o.if_start_point}};
}

// Struct for outputting parsed received messages to handler functions
struct RenderMessage_t {
  SubMessage_t sub_msg;
  std::vector<cv::Mat> images;
};

}  // namespace avoidlib