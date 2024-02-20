#include "avoidlib/bridges/unity_bridge.hpp"

namespace avoidlib {

// constructor
UnityBridge::UnityBridge()
  : client_address_("tcp://*"),
    pub_port_("10255"),
    sub_port_("10256"),
    num_frames_(0),
    last_downloaded_utime_(0),
    last_download_debug_utime_(0),
    u_packet_latency_(0),
    unity_ready_(false) {
  // initialize connections upon creating unity bridge
    initializeConnections();
}

bool UnityBridge::initializeConnections() {
  logger_.info("Initializing ZMQ connection!");

  // create and bind an upload socket
  pub_.set(zmqpp::socket_option::send_high_water_mark, 6);
  pub_.bind(client_address_ + ":" + pub_port_);

  // create and bind a download_socket
  sub_.set(zmqpp::socket_option::receive_high_water_mark, 6);
  sub_.bind(client_address_ + ":" + sub_port_);

  // subscribe all messages from ZMQ
  sub_.subscribe("");

  logger_.info("Initializing ZMQ connections done!");
  return true;
}

bool UnityBridge::connectUnity(const SceneID scene_id) {
  float time_out_count = 0;
  float sleep_useconds = 0.2 * 1e5;
  setScene(scene_id);
  // try to connect unity
  logger_.info("Trying to Connect Unity.");
  std::cout << "[";
  while (!unity_ready_) {
    // if time out
    if (time_out_count / 1e6 > unity_connection_time_out_) {
      std::cout << "]" << std::endl;
      logger_.warn(
        "Unity Connection time out! Make sure that Unity Standalone "
        "or Unity Editor is running the Flightmare.");
      return false;
    }
    // initialize Scene settings
    sendInitialSettings();
    // check if setting is done
    unity_ready_ = handleSettings();
    // sleep
    usleep(sleep_useconds);
    // increase time out counter
    time_out_count += sleep_useconds;
    // print something
    std::cout << ".";
    std::cout.flush();
  }
  logger_.info("Flightmare Unity is connected.");
  return unity_ready_;
}

bool UnityBridge::disconnectUnity() {
  unity_ready_ = false;
  // create new message object
  pub_.close();
  sub_.close();
  return true;
}

bool UnityBridge::sendInitialSettings(void) {
  // create new message object
  zmqpp::message msg;
  // add topic header
  msg << "Pose";
  // create JSON object for initial settings
  json json_mesg = settings_;
  msg << json_mesg.dump();
  // send message without blocking
  pub_.send(msg, true);
  return true;
};

bool UnityBridge::handleSettings(void) {
  // create new message object
  zmqpp::message msg;

  bool done = false;
  // Unpack message metadata
  if (sub_.receive(msg, true)) {
    std::string metadata_string = msg.get(0);
    // Parse metadata
    if (json::parse(metadata_string).size() > 1) {
      return false;  // hack
    }
    done = json::parse(metadata_string).at("ready").get<bool>();
  }
  return done;
};

bool UnityBridge::getRender(const FrameID frame_id, const bool SpawnNewObj) {
  pub_msg_.frame_id = frame_id;
  QuadState quad_state;
  for (size_t idx = 0; idx < pub_msg_.vehicles.size(); idx++) {
    unity_quadrotors_[idx]->getState(&quad_state);
    pub_msg_.vehicles[idx].position = positionRos2Unity(quad_state.p);
    pub_msg_.vehicles[idx].rotation = quaternionRos2Unity(quad_state.q());
  }

  for (size_t idx = 0; idx < pub_msg_.objects.size(); idx++) {
    std::shared_ptr<Cylinder> gate = static_objects_[idx];
    pub_msg_.objects[idx].position = positionRos2Unity(gate->getPosition());
    pub_msg_.objects[idx].rotation = quaternionRos2Unity(gate->getQuaternion());
    pub_msg_.objects[idx].opacity  = gate->getOpacity();
  }

  pub_msg_.spawn_new_obj = SpawnNewObj;
  // create new message object
  zmqpp::message msg;
  // add topic header
  msg << "Pose";
  // create JSON object for pose update and append
  json json_msg = pub_msg_;
  msg << json_msg.dump();
  // send message without blocking
  pub_.send(msg, true);
  return true;
}

bool UnityBridge::setScene(const SceneID& scene_id) {
  if (scene_id >= UnityScene::SceneNum) {
    logger_.warn("Scene ID is not defined, cannot set scene.");
    return false;
  }
  // logger_.info("Scene ID is set to %d.", scene_id);
  settings_.scene_id = scene_id;
  return true;
}

bool UnityBridge::addQuadrotor(std::shared_ptr<Quadrotor> quad) {
  Vehicle_t vehicle_t;
  // get quadrotor state
  QuadState quad_state;
  if (!quad->getState(&quad_state)) {
    logger_.error("Cannot get Quadrotor state.");
    return false;
  }

  vehicle_t.ID = "quadrotor" + std::to_string(settings_.vehicles.size());
  vehicle_t.position = positionRos2Unity(quad_state.p);
  vehicle_t.rotation = quaternionRos2Unity(quad_state.q());
  vehicle_t.size = scalarRos2Unity(quad->getSize());

  // get camera
  std::vector<std::shared_ptr<RGBCamera>> rgb_cameras = quad->getCameras();
  for (size_t cam_idx = 0; cam_idx < rgb_cameras.size(); cam_idx++) {
    std::shared_ptr<RGBCamera> cam = rgb_cameras[cam_idx];
    Camera_t camera_t;
    camera_t.ID = vehicle_t.ID + "_" + std::to_string(cam_idx);
    camera_t.T_BC = transformationRos2Unity(rgb_cameras[cam_idx]->getRelPose());
    camera_t.channels = rgb_cameras[cam_idx]->getChannels();
    camera_t.width = rgb_cameras[cam_idx]->getWidth();
    camera_t.height = rgb_cameras[cam_idx]->getHeight();
    camera_t.fov = rgb_cameras[cam_idx]->getFOV();
    camera_t.depth_scale = rgb_cameras[cam_idx]->getDepthScale();
    camera_t.enabled_layers = rgb_cameras[cam_idx]->getEnabledLayers();
    camera_t.is_depth = false;
    camera_t.output_index = cam_idx;
    vehicle_t.cameras.push_back(camera_t);

    // add rgb_cameras
    rgb_cameras_.push_back(rgb_cameras[cam_idx]);
  }
  unity_quadrotors_.push_back(quad);
  //
  settings_.vehicles.push_back(vehicle_t);
  pub_msg_.vehicles.push_back(vehicle_t);
  return true;
}

std::shared_ptr<Quadrotor> UnityBridge::getQuadrotor(size_t id)
{
  return unity_quadrotors_[id];
}

bool UnityBridge::addStaticObject(std::shared_ptr<Cylinder> static_object) {
  Object_t object_t;
  object_t.ID = static_object->getID();
  object_t.prefab_ID = static_object->getPrefabID();
  object_t.position = positionRos2Unity(static_object->getPosition());
  object_t.rotation = quaternionRos2Unity(static_object->getQuaternion());
  object_t.size = scalarRos2Unity(static_object->getSize());
  object_t.opacity = static_object->getOpacity();

  static_objects_.push_back(static_object);
  settings_.objects.push_back(object_t);
  pub_msg_.objects.push_back(object_t);
  //
  return true;
}

bool UnityBridge::placeObjects(Object_Message_t obj_msg_t) {
  settings_.object_message = obj_msg_t;
  pub_msg_.object_message = obj_msg_t;
  return true;
}

bool UnityBridge::placeTrees(Tree_Message_t obj_msg_t) {
  settings_.tree_message = obj_msg_t;
  pub_msg_.tree_message = obj_msg_t;
  return true;
}

bool UnityBridge::ifSceneChanged()
{
  return scene_changed;
}

bool UnityBridge::handleOutput(const FrameID sent_frame_id) {
  // create new message object
  zmqpp::message msg;
  sub_.receive(msg);
  if(msg.get(0) == "CollisionState")
  {
    // std::cout<<msg.get(1)<<std::endl;
    std::string json_colli_msg = msg.get(1);
    colli_state_msg = json::parse(json_colli_msg);
    return false;
  }
  // unpack message metadata
  std::string json_sub_msg = msg.get(0);
  std::string json_pc_msg = msg.get(1);
  // parse metadata
  SubMessage_t sub_msg = json::parse(json_sub_msg);
  pc_state_msg = json::parse(json_pc_msg);
  scene_changed = sub_msg.scene_change_feedback;
  // std::cout<<scene_changed<<std::endl;
  size_t image_i = 2;
  // ensureBufferIsAllocated(sub_msg);
  collisions.clear();

  for (size_t idx = 0; idx < settings_.vehicles.size(); idx++) {
    // update vehicle collision flag
    unity_quadrotors_[idx]->setCollision(sub_msg.sub_vehicles[idx].collision);
    collisions.push_back(sub_msg.sub_vehicles[idx].collision);
    // feed image data to RGB camera
    for (const auto& cam : settings_.vehicles[idx].cameras) {
      for (size_t layer_idx = 0; layer_idx <= cam.enabled_layers.size();
           layer_idx++) {
        if (!layer_idx == 0 && !cam.enabled_layers[layer_idx - 1]) continue;

        if (layer_idx == 1) {
          // depth
          uint32_t image_len = cam.width * cam.height * 4;
          // Get raw image bytes from ZMQ message.
          // WARNING: This is a zero-copy operation that also casts the input to
          // an array of unit8_t. when the message is deleted, this pointer is
          // also dereferenced.
          const uint8_t* image_data;
          msg.get(image_data, image_i);
          image_i = image_i + 1;
          // Pack image into cv::Mat
          cv::Mat new_image = cv::Mat(cam.height, cam.width, CV_32FC1);
          memcpy(new_image.data, image_data, image_len);
          // Flip image since OpenCV origin is upper left, but Unity's is lower
          // left.
          new_image = new_image * (100.f);
          cv::flip(new_image, new_image, 0);
          unity_quadrotors_[idx]
            ->getCameras()[cam.output_index]
            ->feedImageQueue(layer_idx, new_image);


        } else {
          uint32_t image_len = cam.width * cam.height * cam.channels;
          // Get raw image bytes from ZMQ message.
          // WARNING: This is a zero-copy operation that also casts the input to
          // an array of unit8_t. when the message is deleted, this pointer is
          // also dereferenced.
          const uint8_t* image_data;
          msg.get(image_data, image_i);
          image_i = image_i + 1;
          // Pack image into cv::Mat
          cv::Mat new_image =
            cv::Mat(cam.height, cam.width, CV_MAKETYPE(CV_8U, cam.channels));
          memcpy(new_image.data, image_data, image_len);
          // Flip image since OpenCV origin is upper left, but Unity's is lower
          // left.
          cv::flip(new_image, new_image, 0);

          // Tell OpenCv that the input is RGB.
          if (cam.channels == 3) {
            cv::cvtColor(new_image, new_image, CV_RGB2BGR);
          }
          unity_quadrotors_[idx]
            ->getCameras()[cam.output_index]
            ->feedImageQueue(layer_idx, new_image);
        }
      }
    }
  }
  return true;
}

bool UnityBridge::getPointCloud(PointCloudMessage_t& pointcloud_msg) {
  int ss = 0;
  while(!pc_state_msg.get_pc_msg)
  {
    ss++;
    // create new message object
    zmqpp::message msg;
    // add topic header
    msg << "PointCloud";
    // create JSON object for initial settings
    json json_msg = pointcloud_msg;
    msg << json_msg.dump();
    // send message without blocking
    pub_.send(msg, true);

    usleep(0.01 * 1e6);
  }
  ss = 0;
  pc_state_msg.get_pc_msg = false;
  float wait_time = 0.0f;
  while(!pc_state_msg.save_pc_success)
  {
    ss++;
    usleep(0.2 * 1e6);
  }
  pc_state_msg.save_pc_success = false;
  return true;
}

bool UnityBridge::checkCollisionState(const CollisionCheckMessage_t &collision_check_msg, std::vector<float>* const new_pt)
{
  int ss = 0;
  while(!colli_state_msg.get_msg)
  {
    ss ++;
    zmqpp::message msg;
    // add topic header
    msg << "CollisionCheck";
    // create JSON object for initial settings
    json json_msg = collision_check_msg;
    msg << json_msg.dump();
    // send message without blocking
    pub_.send(msg, true);
    usleep(0.05 * 1e6);
    if(ss>=20) break;
  }
  if(ss==20) return true;
  colli_state_msg.get_msg = false;
  if(colli_state_msg.if_collision)
  {
    *new_pt = colli_state_msg.new_point;
    return true;
  }
  return false;
}

}  // namespace avoidlib
