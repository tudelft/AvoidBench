//
// This bridge was originally from FlightGoggles.
// We made several changes on top of it.
//
#pragma once

// std libs
#include <unistd.h>
#include <experimental/filesystem>
#include <fstream>
#include <map>
#include <string>
#include <unordered_map>

// opencv
#include <opencv2/imgproc/types_c.h>

// Include ZMQ bindings for communications with Unity.
#include <zmqpp/zmqpp.hpp>

// avoidlib
#include "avoidlib/bridges/unity_message_types.hpp"
#include "avoidlib/common/logger.hpp"
#include "avoidlib/common/math.hpp"
#include "avoidlib/common/quad_state.hpp"
#include "avoidlib/common/types.hpp"
#include "avoidlib/objects/quadrotor.hpp"
#include "avoidlib/objects/static_object.hpp"
#include "avoidlib/objects/unity_camera.hpp"
#include "avoidlib/sensors/rgb_camera.hpp"
#include "avoidlib/objects/cylinder.hpp"

using json = nlohmann::json;

namespace avoidlib {

class UnityBridge {
 public:
  // constructor & destructor
  UnityBridge();
  ~UnityBridge(){};

  // connect function
  bool connectUnity(const SceneID scene_id);
  bool disconnectUnity(void);

  // public get functions
  bool getRender(const FrameID frame_id, const bool SpawnNewObj = false);
  bool handleOutput(const FrameID sent_frame_id);
  bool getPointCloud(PointCloudMessage_t &pointcloud_msg);
  bool checkCollisionState(const CollisionCheckMessage_t &collision_check_msg, std::vector<float>* const new_pt);
  // public set functions
  bool setScene(const SceneID &scene_id);

  // add object
  bool addQuadrotor(std::shared_ptr<Quadrotor> quad);
  std::shared_ptr<Quadrotor> getQuadrotor(size_t id);
  bool addCamera(std::shared_ptr<UnityCamera> unity_camera);
  bool addStaticObject(std::shared_ptr<Cylinder> static_object);

  bool placeObjects(Object_Message_t obj_msg_t);
  bool placeTrees(Tree_Message_t obj_msg_t);
  bool ifSceneChanged();
  std::vector<bool> collisions;

  // public auxiliary functions
  inline void setPubPort(const std::string &pub_port) { pub_port_ = pub_port; };
  inline void setSubPort(const std::string &sub_port) { sub_port_ = sub_port; };
  // create unity bridge
  static std::shared_ptr<UnityBridge> getInstance(void) {
    static std::shared_ptr<UnityBridge> bridge_ptr =
      std::make_shared<UnityBridge>();
    return bridge_ptr;
  };
  bool initializeConnections(void);

 private:

  //
  SettingsMessage_t settings_;
  PubMessage_t pub_msg_;
  sub_pointcloud_state pc_state_msg;
  sub_collision_state colli_state_msg;
  Logger logger_{"UnityBridge"};

  std::vector<std::shared_ptr<Quadrotor>> unity_quadrotors_;
  std::vector<std::shared_ptr<RGBCamera>> rgb_cameras_;
  std::vector<std::shared_ptr<Cylinder>> static_objects_;

  // ZMQ variables and functions
  std::string client_address_;
  std::string pub_port_;
  std::string sub_port_;
  zmqpp::context context_;
  zmqpp::socket pub_{context_, zmqpp::socket_type::publish};
  zmqpp::socket sub_{context_, zmqpp::socket_type::subscribe};
  bool sendInitialSettings(void);
  bool handleSettings(void);
  bool scene_changed{false};

  // timing variables
  int64_t num_frames_;
  int64_t last_downloaded_utime_;
  int64_t last_download_debug_utime_;
  int64_t u_packet_latency_;

  // axuiliary variables
  const float unity_connection_time_out_{60.0};
  bool unity_ready_{false};
};
}  // namespace avoidlib