#include "avoidlib/envs/env_base.hpp"

namespace avoidlib {

EnvBase::EnvBase()
  : obs_dim_(0),
    act_dim_(0),
    state_dim_(0),
    rew_dim_(0),
    img_width_(0),
    img_height_(0),
    sim_dt_(0.0),
    max_t_(0.0) {}

EnvBase::~EnvBase() {}

bool EnvBase::getImage(Ref<ImgVector<>> img, const bool rgb) { return false; }

bool EnvBase::getDepthImage(Ref<DepthImgVector<>> img) { return false; }

bool EnvBase::addQuadrotorToUnity(const std::shared_ptr<UnityBridge> bridge) {
  return false;
}

void EnvBase::curriculumUpdate() {}

void EnvBase::close() {}

void EnvBase::render() {}

void EnvBase::updateExtraInfo() {}

bool EnvBase::isTerminalState(double &reward) {
  reward = 0.f;
  return false;
}

}  // namespace avoidlib
