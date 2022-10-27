#pragma once

#include "avoidlib/bridges/unity_bridge.hpp"
#include "sgm_gpu/sgm_gpu.h"
#include "quadrotor_common/quad_state_estimate.h"
#include "quadrotor_common/trajectory.h"
#include <sys/time.h>

namespace avoidlib {

struct mission_parameter
{
    /* data */
    std::vector<float> m_start_point;
    std::vector<float> m_end_point;
    float m_radius;
    float m_opacity;
    int m_seed;
    int trials;
    std::string m_pc_file_name;
};

class AvoidbenchBridge {
public:
    AvoidbenchBridge(const std::string &cfg_path);
    ~AvoidbenchBridge() {};
    
    // for python wrapper
    bool finish_check{false};
    bool finish_pc_save{false};

    bool updateUnity(const quadrotor_common::QuadStateEstimate& start_state);
    void getImages(cv::Mat* left_frame, cv::Mat* depth_uint16);
    void getImages(cv::Mat* left_frame, cv::Mat* right_frame, 
                    cv::Mat* depth_uint16, cv::Mat* disp_uint8);
    void getPointCloud(const std::string curr_data_dir,
                        const Eigen::Vector3d &range, const Eigen::Vector3d &origin);
    bool checkCollisionState(std::vector<float>* const point, const bool &if_start=false);
    bool connectUnity();
    void setParamFromMission(const mission_parameter &para);
    void SpawnNewObs();
    bool ifSceneChanged();
    bool spawnObstacles();
    bool getQuadCollisionState();

private:
    bool loadParameters(const YAML::Node &cfg);
    bool setUnity(const bool render);
    void computeDepthImage(const cv::Mat& left_frame, const cv::Mat& right_frame, cv::Mat* const depth, cv::Mat* const disp);
    bool unity_render_{false};
    bool unity_ready_{false};

    std::shared_ptr<sgm_gpu::SgmGpu> sgm_;
    std::shared_ptr<Quadrotor> quad_ptr_;
    std::shared_ptr<RGBCamera> right_rgb_cam_, left_rgb_cam_;
    std::shared_ptr<UnityBridge> unity_bridge_ptr_;
    SceneID scene_id_{UnityScene::WAREHOUSE};
    QuadState quad_state_;

    //camera parameters
    float stereo_baseline_;
    float pitch_angle_deg_;
    float rgb_fov_deg_;
    int img_cols_, img_rows_;
    bool perform_sgm_;
    //unity parameters
    int env_idx_;
    bool spawn_trees_;
    bool spawn_objects_;
    bool spawn_new_;
    std::vector<float> bounding_box_;
    std::vector<float> bounding_box_origin_;
    int seed;
    float radius;
    float opacity;
    std::vector<float> min_object_scale;
    std::vector<float> max_object_scale;
    //point cloud
    bool if_get_pointcloud_;
    std::vector<float> range_;
    std::vector<float> origin_;
    float res_;
    std::string file_name_;
};
}