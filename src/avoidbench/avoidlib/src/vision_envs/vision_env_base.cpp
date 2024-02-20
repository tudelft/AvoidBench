#include "avoidlib/vision_envs/vision_env_base.h"

namespace avoidlib {

  VisionEnvBase::VisionEnvBase()
  : obs_dim_(0),
    act_dim_(0),
    state_dim_(0),
    rew_dim_(0),
    img_width_(0),
    img_height_(0),
    sim_dt_(0.0),
    max_t_(0.0) {}

VisionEnvBase::~VisionEnvBase() {}

bool VisionEnvBase::getImage(Ref<ImgVector<>> img, const bool rgb) { return false; }

bool VisionEnvBase::getDepthImage(Ref<DepthImgVector<>> img) { return false; }


bool VisionEnvBase::addQuadrotorToUnity(const std::shared_ptr<UnityBridge> bridge) {
  return false;
}

bool VisionEnvBase::setPointClouds(const std::shared_ptr<Environment> env_ptr) {
  return false;
}

void VisionEnvBase::setTraversability(double traversability) {}

void VisionEnvBase::setQuadFromPtr(const std::shared_ptr<UnityBridge> bridge) {}

void VisionEnvBase::close() {}

void VisionEnvBase::render() {}

void VisionEnvBase::updateExtraInfo() {}

bool VisionEnvBase::isTerminalState(double &reward) {
  reward = 0.f;
  return false;
}
}