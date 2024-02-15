
// pybind11
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

// avoidlib
#include "avoidlib/envs/env_base.hpp"
#include "avoidlib/envs/quadrotor_env/quadrotor_env.hpp"
#include "avoidlib/envs/quadrotor_env/quadrotor_vec_env.hpp"
#include "avoidlib/envs/vec_env_base.hpp"

#include "avoidlib/vision_envs/vision_env_base.h"
#include "avoidlib/vision_envs/vec_vision_env_base.h"
#include "avoidlib/vision_envs/avoid_vision_envs/avoid_vision_envs.h"
#include "avoidlib/vision_envs/avoid_vision_envs/avoid_vec_vision_envs.h"

namespace py = pybind11;
using namespace avoidlib;

void PointCloudThread(AvoidVecVisionEnv<AvoidVisionEnv>& vec_env, std::string data_dir, int id, bool save_pc)
{
  vec_env.getPointClouds(data_dir, id, save_pc);
}

void PointCloudReadingThread(AvoidVecVisionEnv<AvoidVisionEnv>& vec_env, int id)
{
  vec_env.readPointClouds(id);
}

PYBIND11_MODULE(flightgym, m) {
  py::class_<UnityBridge, std::shared_ptr<UnityBridge>>(m, "UnityBridge")
    .def(py::init<>());

  py::class_<QuadrotorVecEnv<QuadrotorEnv>>(m, "QuadrotorEnv_v1")
    .def(py::init<>())
    .def(py::init<const std::string&>())
    .def(py::init<const std::string&, const bool>())
    .def("reset",
         static_cast<bool (QuadrotorVecEnv<QuadrotorEnv>::*)(
           Ref<MatrixRowMajor<>>)>(&QuadrotorVecEnv<QuadrotorEnv>::reset),
         "reset")
    .def("reset",
         static_cast<bool (QuadrotorVecEnv<QuadrotorEnv>::*)(
           Ref<MatrixRowMajor<>>, Ref<MatrixRowMajor<>>, Ref<MatrixRowMajor<>>)>(&QuadrotorVecEnv<QuadrotorEnv>::reset),
         "reset")
    .def("reset",
         static_cast<bool (QuadrotorVecEnv<QuadrotorEnv>::*)(
           Ref<MatrixRowMajor<>>, bool)>(&QuadrotorVecEnv<QuadrotorEnv>::reset),
         "reset with random option")
    .def("step", &QuadrotorVecEnv<QuadrotorEnv>::step)
    .def("setSeed", &QuadrotorVecEnv<QuadrotorEnv>::setSeed)
    .def("close", &QuadrotorVecEnv<QuadrotorEnv>::close)
    .def("isTerminalState", &QuadrotorVecEnv<QuadrotorEnv>::isTerminalState)
    .def("curriculumUpdate", &QuadrotorVecEnv<QuadrotorEnv>::curriculumUpdate)
    .def("connectUnity", &QuadrotorVecEnv<QuadrotorEnv>::connectUnity)
    .def("disconnectUnity", &QuadrotorVecEnv<QuadrotorEnv>::disconnectUnity)
    .def("updateUnity", &QuadrotorVecEnv<QuadrotorEnv>::updateUnity)
    .def("getObs", &QuadrotorVecEnv<QuadrotorEnv>::getObs)
    .def("getQuadAct", &QuadrotorVecEnv<QuadrotorEnv>::getQuadAct)
    .def("getQuadState", &QuadrotorVecEnv<QuadrotorEnv>::getQuadState)
    .def("getImage", &QuadrotorVecEnv<QuadrotorEnv>::getImage)
    .def("getDepthImage", &QuadrotorVecEnv<QuadrotorEnv>::getDepthImage)
    .def("getNumOfEnvs", &QuadrotorVecEnv<QuadrotorEnv>::getNumOfEnvs)
    .def("getObsDim", &QuadrotorVecEnv<QuadrotorEnv>::getObsDim)
    .def("getActDim", &QuadrotorVecEnv<QuadrotorEnv>::getActDim)
    .def("getStateDim", &QuadrotorVecEnv<QuadrotorEnv>::getStateDim)
    .def("getMotorDim", &QuadrotorVecEnv<QuadrotorEnv>::getMotorDim)
    .def("getRewDim", &QuadrotorVecEnv<QuadrotorEnv>::getRewDim)
    .def("getImgHeight", &QuadrotorVecEnv<QuadrotorEnv>::getImgHeight)
    .def("getImgWidth", &QuadrotorVecEnv<QuadrotorEnv>::getImgWidth)
    .def("getRewardNames", &QuadrotorVecEnv<QuadrotorEnv>::getRewardNames)
    .def("getExtraInfoNames", &QuadrotorVecEnv<QuadrotorEnv>::getExtraInfoNames)
    .def("__repr__", [](const QuadrotorVecEnv<QuadrotorEnv>& a) {
      return "RPG Drone Control Environment";
    });

    py::class_<AvoidVecVisionEnv<AvoidVisionEnv>>(m, "AvoidVisionEnv_v1")
    .def(py::init<>())
    .def(py::init<const std::string&>())
    .def(py::init<const std::string&, const bool>())
    .def("reset",
         static_cast<bool (AvoidVecVisionEnv<AvoidVisionEnv>::*)(
           Ref<MatrixRowMajor<>>)>(&AvoidVecVisionEnv<AvoidVisionEnv>::reset),
         "reset")
    .def("reset",
         static_cast<bool (AvoidVecVisionEnv<AvoidVisionEnv>::*)(
           Ref<MatrixRowMajor<>>, bool)>(&AvoidVecVisionEnv<AvoidVisionEnv>::reset),
         "reset with random option")
    .def("resetRewCoeff", &AvoidVecVisionEnv<AvoidVisionEnv>::resetRewCoeff)
    .def("step", &AvoidVecVisionEnv<AvoidVisionEnv>::step)
    .def("close", &AvoidVecVisionEnv<AvoidVisionEnv>::close)
    .def("setSeed", &AvoidVecVisionEnv<AvoidVisionEnv>::setSeed)
    .def("isTerminalState", &AvoidVecVisionEnv<AvoidVisionEnv>::isTerminalState)
    .def("connectUnity", &AvoidVecVisionEnv<AvoidVisionEnv>::connectUnity)
    .def("initializeConnections", &AvoidVecVisionEnv<AvoidVisionEnv>::initializeConnections)
    .def("disconnectUnity", &AvoidVecVisionEnv<AvoidVisionEnv>::disconnectUnity)
    .def("updateUnity", &AvoidVecVisionEnv<AvoidVisionEnv>::updateUnity)
    .def("getPointClouds", [](AvoidVecVisionEnv<AvoidVisionEnv>& vec_env, std::string data_dir, int id, bool save_pc) {
      std::thread PCThread(PointCloudThread, std::ref(vec_env), data_dir, id, save_pc);
      PCThread.detach();
    })
    .def("readPointClouds", [](AvoidVecVisionEnv<AvoidVisionEnv>& vec_env, int id) {
      std::thread PCReadingThread(PointCloudReadingThread, std::ref(vec_env), id);
      PCReadingThread.detach();
    })
    // .def("readPointClouds", &AvoidVecVisionEnv<AvoidVisionEnv>::readPointClouds)
    .def("setUnityFromPtr", &AvoidVecVisionEnv<AvoidVisionEnv>::setUnityFromPtr)
    .def("getUnityPtr", &AvoidVecVisionEnv<AvoidVisionEnv>::getUnityPtr)
    .def("getSavingState", &AvoidVecVisionEnv<AvoidVisionEnv>::getSavingState)
    .def("getReadingState", &AvoidVecVisionEnv<AvoidVisionEnv>::getReadingState)
    .def("getObs", &AvoidVecVisionEnv<AvoidVisionEnv>::getObs)
    .def("getQuadAct", &AvoidVecVisionEnv<AvoidVisionEnv>::getQuadAct)
    .def("getQuadState", &AvoidVecVisionEnv<AvoidVisionEnv>::getQuadState)
    .def("getImage", &AvoidVecVisionEnv<AvoidVisionEnv>::getImage)
    .def("getDepthImage", &AvoidVecVisionEnv<AvoidVisionEnv>::getDepthImage)
    .def("spawnObstacles", &AvoidVecVisionEnv<AvoidVisionEnv>::spawnObstacles)
    .def("ifSceneChanged", &AvoidVecVisionEnv<AvoidVisionEnv>::ifSceneChanged)
    .def("getNumOfEnvs", &AvoidVecVisionEnv<AvoidVisionEnv>::getNumOfEnvs)
    .def("getObsDim", &AvoidVecVisionEnv<AvoidVisionEnv>::getObsDim)
    .def("getActDim", &AvoidVecVisionEnv<AvoidVisionEnv>::getActDim)
    .def("getSeqDim", &AvoidVecVisionEnv<AvoidVisionEnv>::getSeqDim)
    .def("getGoalObsDim", &AvoidVecVisionEnv<AvoidVisionEnv>::getGoalObsDim)
    .def("getStateDim", &AvoidVecVisionEnv<AvoidVisionEnv>::getStateDim)
    .def("getRewDim", &AvoidVecVisionEnv<AvoidVisionEnv>::getRewDim)
    .def("getImgHeight", &AvoidVecVisionEnv<AvoidVisionEnv>::getImgHeight)
    .def("getImgWidth", &AvoidVecVisionEnv<AvoidVisionEnv>::getImgWidth)
    .def("getRewardNames", &AvoidVecVisionEnv<AvoidVisionEnv>::getRewardNames)
    .def("getExtraInfoNames", &AvoidVecVisionEnv<AvoidVisionEnv>::getExtraInfoNames)
    .def("__repr__", [](const AvoidVecVisionEnv<AvoidVisionEnv>& a) {
      return "AvoidBench learning obstacle avoidance Environment";
    });

}
