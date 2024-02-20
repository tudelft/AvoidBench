// pybind11
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include<pybind11/numpy.h>
#include <thread>

#include "avoidlib/bridges/avoidbench_bridge.hpp"

#include "avoidlib/envs/env_base.hpp"
#include "avoidlib/envs/quadrotor_env/quadrotor_env.hpp"
#include "avoidlib/envs/quadrotor_env/quadrotor_vec_env.hpp"
#include "avoidlib/envs/vec_env_base.hpp"

namespace py = pybind11;
using namespace avoidlib;
using namespace quadrotor_common;

std::vector<float> new_point;
bool checking_result;

void checkPointThread(AvoidbenchBridge& bridge, std::vector<float> pt, bool if_start)
{
  checking_result = bridge.checkCollisionState(&pt, if_start);
  new_point = pt;
}

void PointCloudThread(AvoidbenchBridge& bridge, std::string data_dir, Eigen::Vector3d range, Eigen::Vector3d origin)
{
  bridge.getPointCloud(data_dir, range, origin);
}

void setStateEstimate(QuadStateEstimate& state, py::array_t<double> pos, py::array_t<double> ori, py::array_t<double> vel, double time)
{
  py::buffer_info pos_buf = pos.request();
  py::buffer_info ori_buf = ori.request();
  py::buffer_info vel_buf = vel.request();
  double* pos_ptr = (double*)pos_buf.ptr;
  double* ori_ptr = (double*)ori_buf.ptr;
  double* vel_ptr = (double*)vel_buf.ptr;

  state.position.x() = pos_ptr[0];
  state.position.y() = pos_ptr[1];
  state.position.z() = pos_ptr[2];
  state.velocity.x() = vel_ptr[0];
  state.velocity.y() = vel_ptr[1];
  state.velocity.z() = vel_ptr[2];
  state.orientation.x() = ori_ptr[0];
  state.orientation.y() = ori_ptr[1];
  state.orientation.z() = ori_ptr[2];
  state.orientation.w() = ori_ptr[3];
  state.timestamp = ros::Time(time);
  // std::cout<<pos_ptr[0]<<" "<<pos_ptr[1]<<" "<<pos_ptr[2]<<std::endl;
}

PYBIND11_MODULE(avoidbridge, m) {
  py::class_<AvoidbenchBridge, std::shared_ptr<AvoidbenchBridge>>(m, "AvoidbenchBridge")
    .def(py::init<const std::string&>())
    .def("updateUnity", &AvoidbenchBridge::updateUnity)
    .def("getQuadCollisionState", &AvoidbenchBridge::getQuadCollisionState)
    .def("setParamFromMission", &AvoidbenchBridge::setParamFromMission)
    .def("spawnObstacles", &AvoidbenchBridge::spawnObstacles)
    .def("ifSceneChanged", &AvoidbenchBridge::ifSceneChanged)
    .def("SpawnNewObs", &AvoidbenchBridge::SpawnNewObs)
    .def("checkCollisionState", [](AvoidbenchBridge& bridge, std::vector<float> point, bool if_start) {
      std::thread checkThread(checkPointThread, std::ref(bridge), point, if_start);
      checkThread.detach();
    })
    .def("getPointCloud", [](AvoidbenchBridge& bridge, std::string data_dir, Eigen::Vector3d range, Eigen::Vector3d origin) {
      std::thread PCThread(PointCloudThread, std::ref(bridge), data_dir, range, origin);
      PCThread.detach();
    })
    .def("getImages", [](AvoidbenchBridge& bridge) {
      cv::Mat left_cv, right_cv, depth_cv, disp_cv;
      bridge.getImages(&left_cv, &right_cv, &depth_cv, &disp_cv);
      py::array_t<unsigned char> left_py = py::array_t<unsigned char>({ left_cv.rows,left_cv.cols,3}, left_cv.data);
      py::array_t<unsigned char> right_py = py::array_t<unsigned char>({ right_cv.rows,right_cv.cols,3}, right_cv.data);
      py::array_t<unsigned short> depth_py = py::array_t<unsigned short>({ depth_cv.rows,depth_cv.cols,1}, (ushort*)depth_cv.data);
      // py::array_t<unsigned char> disp_py = py::array_t<unsigned char>({ disp_cv.rows,disp_cv.cols,1}, disp_cv.data);
      std::tuple<py::array_t<unsigned char>, py::array_t<unsigned char>, py::array_t<unsigned short>> images
                                                                                (left_py, right_py, depth_py);
      return images;
    })
    .def_readwrite("finish_check", &AvoidbenchBridge::finish_check)
    .def_readwrite("finish_pc_save", &AvoidbenchBridge::finish_pc_save);

  m.def("getCheckingResult", [](){
      return checking_result;
    });
  
  py::class_<QuadStateEstimate>(m, "quadStateEstimate")
    .def(py::init<>())
    .def(py::init<const nav_msgs::Odometry&>())
    .def("transformVelocityToWorldFrame", &QuadStateEstimate::transformVelocityToWorldFrame)
    .def("setStateEstimate", &setStateEstimate)
    .def_readwrite("timestamp", &QuadStateEstimate::timestamp)
    .def_readwrite("position", &QuadStateEstimate::position)
    .def_readwrite("velocity", &QuadStateEstimate::velocity);

  py::class_<mission_parameter>(m, "mission_parameter")
    .def(py::init<>())
    .def_readwrite("m_start_point", &mission_parameter::m_start_point)
    .def_readwrite("m_end_point", &mission_parameter::m_end_point)
    .def_readwrite("m_radius", &mission_parameter::m_radius)
    .def_readwrite("m_opacity", &mission_parameter::m_opacity)
    .def_readwrite("m_seed", &mission_parameter::m_seed)
    .def_readwrite("trials", &mission_parameter::trials)
    .def_readwrite("m_pc_file_name", &mission_parameter::m_pc_file_name);
    
}
