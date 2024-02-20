#include "avoidlib/bridges/avoidbench_bridge.hpp"

namespace avoidlib {

AvoidbenchBridge::AvoidbenchBridge(const std::string &cfg_path)
    : unity_render_(false),
    unity_ready_ (false),
    spawn_new_(false)
{
  std::cout<<cfg_path<<std::endl;
  YAML::Node cfg_ = YAML::LoadFile(cfg_path);
  if (!loadParameters(cfg_)) {
    std::cout<<"load parameter failed 1"<<std::endl;
  }
  // setParamFromMission(para);
  quad_ptr_ = std::make_shared<Quadrotor>();

  float hor_fov_radians = (M_PI * (rgb_fov_deg_ / 180.0));
  float avoidbench_fov = 2. * std::atan(std::tan(hor_fov_radians / 2) * img_rows_ / img_cols_);
  avoidbench_fov = (avoidbench_fov / M_PI) * 180.0;

  left_rgb_cam_ = std::make_shared<RGBCamera>();
  left_rgb_cam_ ->setFOV(avoidbench_fov);
  left_rgb_cam_->setWidth(img_cols_);
  left_rgb_cam_->setHeight(img_rows_);
  if(perform_sgm_)
    left_rgb_cam_->setPostProcessing(std::vector<bool>{false, false, false});
  else
    left_rgb_cam_->setPostProcessing(std::vector<bool>{true, false, false});
  Vector<3> B_r_BCl(0.2, stereo_baseline_ / 2.0, 0.3);
  Matrix<3, 3> R_BCl;
  R_BCl = Eigen::AngleAxisd(0.0 * M_PI, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(-pitch_angle_deg_ / 180.0 * M_PI,
                          Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ());
  left_rgb_cam_->setRelPose(B_r_BCl, R_BCl);
  quad_ptr_->addRGBCamera(left_rgb_cam_);
  
  if(perform_sgm_)
  {
    right_rgb_cam_ = std::make_shared<RGBCamera>();
    right_rgb_cam_ ->setFOV(avoidbench_fov);
    right_rgb_cam_->setWidth(img_cols_);
    right_rgb_cam_->setHeight(img_rows_);
    right_rgb_cam_->setPostProcessing(std::vector<bool>{false, false, false});
    Vector<3> B_r_BCr(0.2, -stereo_baseline_ / 2.0, 0.3);
    Matrix<3, 3> R_BCr; 
    R_BCr = Eigen::AngleAxisd(0.0 * M_PI, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(-pitch_angle_deg_ / 180.0 * M_PI,
                            Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ());
    right_rgb_cam_->setRelPose(B_r_BCr, R_BCr);
    quad_ptr_->addRGBCamera(right_rgb_cam_);
  }

  quad_state_.setZero();
  quad_state_.q(Eigen::Quaterniond(std::cos(0.5 * M_PI_2), 0.0, 0.0,
                                  std::sin(0.5 * M_PI_2)));
  quad_ptr_->reset(quad_state_);
  unity_bridge_ptr_ = UnityBridge::getInstance();
  unity_bridge_ptr_->addQuadrotor(quad_ptr_);
  sgm_.reset(new sgm_gpu::SgmGpu(img_cols_, img_rows_));
  // ros::Duration(2.0).sleep();
  usleep(2.0*1e6);

  scene_id_ = env_idx_;
  connectUnity();

}

bool AvoidbenchBridge::loadParameters(const YAML::Node &cfg)
{
    if (cfg["camera"] && cfg["unity"])
    {
      rgb_fov_deg_ = cfg["camera"]["fov"].as<Scalar>();
      img_cols_ = cfg["camera"]["width"].as<int>();
      img_rows_ = cfg["camera"]["height"].as<int>();
      stereo_baseline_ = cfg["camera"]["baseline"].as<double>();
      pitch_angle_deg_ = cfg["camera"]["pitch_angle_deg"].as<double>();
      perform_sgm_ = cfg["camera"]["perform_sgm"].as<bool>();

      unity_render_ = cfg["unity"]["unity_render"].as<bool>();
      spawn_trees_ = cfg["unity"]["spawn_trees"].as<bool>();
      spawn_objects_ = cfg["unity"]["spawn_objects"].as<bool>();
      env_idx_ = cfg["unity"]["env_idx"].as<int>();
      bounding_box_ = cfg["unity"]["bounding_box"].as<std::vector<float>>();
      bounding_box_origin_ = cfg["unity"]["bounding_box_origin"].as<std::vector<float>>();
      min_object_scale = cfg["unity"]["min_object_scale"].as<std::vector<float>>();
      max_object_scale = cfg["unity"]["max_object_scale"].as<std::vector<float>>();

      if_get_pointcloud_ = cfg["unity"]["if_get_pointcloud"].as<bool>();
      range_ = cfg["unity"]["range"].as<std::vector<float>>();
      origin_ = cfg["unity"]["origin"].as<std::vector<float>>();
      res_ = cfg["unity"]["res"].as<float>();
      
      return true;
    }
    else return false;
}

void AvoidbenchBridge::setParamFromMission(const mission_parameter &para)
{
  seed = para.m_seed;
  radius = para.m_radius;
  opacity = para.m_opacity;
  file_name_ = para.m_pc_file_name;
}

bool AvoidbenchBridge::spawnObstacles()
{
    if(spawn_trees_)
    {
        Tree_Message_t trees;
        trees.name = "trees";
        trees.bounding_area[0] = bounding_box_[0];
        trees.bounding_area[1] = bounding_box_[1];
        trees.bounding_origin[0] = bounding_box_origin_[0];
        trees.bounding_origin[1] = bounding_box_origin_[1];
        trees.seed = seed;
        trees.radius = radius;
        unity_bridge_ptr_->placeTrees(trees);
    }
    if(spawn_objects_)
    {
        Object_Message_t objects;
        objects.name = "cylinder";
        objects.bounding_area[0] = bounding_box_[0];
        objects.bounding_area[1] = bounding_box_[1];
        objects.bounding_origin[0] = bounding_box_origin_[0];
        objects.bounding_origin[1] = bounding_box_origin_[1];
        objects.scale_min = min_object_scale;
        objects.scale_max = max_object_scale;
        objects.seed = seed;
        objects.radius = radius;
        objects.opacity = opacity;
        unity_bridge_ptr_->placeObjects(objects);
    }

    return true;
}

bool AvoidbenchBridge::connectUnity() {
  if (!unity_render_ || unity_bridge_ptr_ == nullptr) return false;
  unity_ready_ = unity_bridge_ptr_->connectUnity(scene_id_);
  return unity_ready_;
}

bool AvoidbenchBridge::updateUnity(const quadrotor_common::QuadStateEstimate& start_state) {
    quad_state_.x[QS::POSX] = (Scalar)start_state.position.x();
    quad_state_.x[QS::POSY] = (Scalar)start_state.position.y();
    quad_state_.x[QS::POSZ] = (Scalar)start_state.position.z();
    Eigen::Matrix3d rot = start_state.orientation.toRotationMatrix()*Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond rot_q(rot);
    quad_state_.x[QS::ATTW] = (Scalar)rot_q.w();
    quad_state_.x[QS::ATTX] = (Scalar)rot_q.x();
    quad_state_.x[QS::ATTY] = (Scalar)rot_q.y();
    quad_state_.x[QS::ATTZ] = (Scalar)rot_q.z();
    // quad_state_.x[QS::ATTW] = (Scalar)start_state.orientation.w();
    // quad_state_.x[QS::ATTX] = (Scalar)start_state.orientation.x();
    // quad_state_.x[QS::ATTY] = (Scalar)start_state.orientation.y();
    // quad_state_.x[QS::ATTZ] = (Scalar)start_state.orientation.z();

    quad_ptr_->setState(quad_state_);

    if (unity_render_ && unity_ready_) {
        unity_bridge_ptr_->getRender(0, spawn_new_);
        spawn_new_ = false;
        while(!unity_bridge_ptr_->handleOutput(0)) {usleep(0.005*1e6);}
        if (quad_ptr_->getCollision()) {
        // collision happened
        }
        return true;
    }
    return false;
}

void AvoidbenchBridge::SpawnNewObs()
{
  spawn_new_ = true;
}

bool AvoidbenchBridge::ifSceneChanged()
{
  return unity_bridge_ptr_->ifSceneChanged();
}

void AvoidbenchBridge::getImages(cv::Mat* left_frame, cv::Mat* depth_uint16)
{
  left_rgb_cam_->getRGBImage(*left_frame);
  left_rgb_cam_->getDepthMap(*depth_uint16);
}

void AvoidbenchBridge::getImages(cv::Mat* left_frame, cv::Mat* right_frame, 
                                  cv::Mat* depth_uint16, cv::Mat* disp_uint8) {
  if(!perform_sgm_) 
  {
    std::cout<<"must be stereo vision"<<std::endl;
    return;
  }
  left_rgb_cam_->getRGBImage(*left_frame);
  right_rgb_cam_->getRGBImage(*right_frame);
  // compute disparity image
  *depth_uint16 = cv::Mat(img_rows_, img_cols_, CV_16UC1);
  *disp_uint8   = cv::Mat(img_rows_, img_cols_, CV_8UC1);
  computeDepthImage(*left_frame, *right_frame, depth_uint16, disp_uint8);
}

void AvoidbenchBridge::computeDepthImage(const cv::Mat& left_frame,
                                         const cv::Mat& right_frame,
                                         cv::Mat* const depth, cv::Mat* const disp) {
  // ros::WallTime start_disp_comp = ros::WallTime::now();
  cv::Mat disparity(img_rows_, img_cols_, CV_8UC1);
  sgm_->computeDisparity(left_frame, right_frame, disp);
  disp->copyTo(disparity);
  disparity.convertTo(disparity, CV_32FC1);

  // compute depth from disparity
  cv::Mat depth_float(img_rows_, img_cols_, CV_32FC1);

  float f = (img_cols_ / 2.0) / std::tan((M_PI * (rgb_fov_deg_ / 180.0)) / 2.0);
  //  depth = static_cast<float>(stereo_baseline_) * f / disparity;
  for (int r = 0; r < img_rows_; ++r) {
    for (int c = 0; c < img_cols_; ++c) {
      if (disparity.at<float>(r, c) == 0.0f) {
        depth_float.at<float>(r, c) = 0.0f;
        depth->at<unsigned short>(r, c) = 0;
      } else if (disparity.at<float>(r, c) == 255.0f) {
        depth_float.at<float>(r, c) = 0.0f;
        depth->at<unsigned short>(r, c) = 0;
      } else {
        depth_float.at<float>(r, c) = static_cast<float>(stereo_baseline_) * f /
                                      disparity.at<float>(r, c);
        depth->at<unsigned short>(r, c) = static_cast<unsigned short>(
            1000.0 * static_cast<float>(stereo_baseline_) * f /
            disparity.at<float>(r, c));
      }
    }
  }
  // double disp_comp_duration = (ros::WallTime::now() - start_disp_comp).toSec();
}

void AvoidbenchBridge::getPointCloud(const std::string curr_data_dir, 
                                      const Eigen::Vector3d &range, const Eigen::Vector3d &origin)
{
  finish_pc_save = false; //"finish_check" just used for python side because of the multi thread
  PointCloudMessage_t pc_msg;
  pc_msg.range[0] = range.x();
  pc_msg.range[1] = range.y();
  pc_msg.range[2] = range.z();
  pc_msg.origin[0] = origin.x();
  pc_msg.origin[1] = origin.y();
  pc_msg.origin[2] = origin.z();
  if(curr_data_dir != "")
    pc_msg.path = curr_data_dir;
  pc_msg.file_name = file_name_;
  unity_bridge_ptr_->getPointCloud(pc_msg);
  finish_pc_save = true;
}

bool AvoidbenchBridge::checkCollisionState(std::vector<float>* const point, const bool &if_start)
{
  finish_check = false; //"finish_check" just used for python side because of the multi thread
  CollisionCheckMessage_t collision_check_msg;
  collision_check_msg.checked_point = *point;
  collision_check_msg.if_start_point = if_start;
  if(if_start)
    collision_check_msg.drone_width = 1.0;
  bool is_collision = unity_bridge_ptr_->checkCollisionState(collision_check_msg, point);
  finish_check = true;
  return is_collision;
}

bool AvoidbenchBridge::getQuadCollisionState()
{
  if (unity_bridge_ptr_->collisions.size()!=0)
  {
    return unity_bridge_ptr_->collisions[0];
  }
    
  else return false;
}

}