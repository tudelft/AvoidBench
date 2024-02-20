#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <thread>

#include "Metrics.h"

namespace py = pybind11;

void MetricsRunThread(avoidmetrics::Metrics& metrics)
{
  metrics.run();
}

PYBIND11_MODULE(avoidmetrics, m) {
  py::class_<avoidmetrics::Metrics, std::shared_ptr<avoidmetrics::Metrics>>(m, "AvoidMetrics")
  .def(py::init<const std::string&>())
  .def("getMetricsMsg", [](avoidmetrics::Metrics& metrics){
    avoid_msgs::Metrics msg = metrics.getMetricsMsg();
    std::vector<avoidmetrics::Metrics_msg> msgs_py;
    for(int i=0; i<msg.factors.size(); i++)
    {
      avoidmetrics::Metrics_msg msg_py;
      msg_py.traversability = msg.factors[i].traversability;
      msg_py.relative_gap_size = msg.factors[i].relative_gap_size;
      for(int j=0; j<msg.factors[i].optimality_factor.size(); j++)
      {
        msg_py.optimality_factor.push_back(msg.factors[i].optimality_factor[j]);
        msg_py.average_goal_velocity.push_back(msg.factors[i].average_goal_velocity[j]);
        msg_py.mission_progress.push_back(msg.factors[i].mission_progress[j]);
        msg_py.processing_time.push_back(msg.factors[i].processing_time[j]);
        msg_py.collision_number.push_back(msg.factors[i].collision_number[j]);
      }
      msgs_py.push_back(msg_py);
    }
    return msgs_py;
  })
  .def("setMissions", &avoidmetrics::Metrics::setMissions)
  .def("setTaskFinishFlag", &avoidmetrics::Metrics::setTaskFinishFlag)
  .def_readwrite("task_finished", &avoidmetrics::Metrics::task_finished)
  .def("run", [](avoidmetrics::Metrics& metrics) {
    std::thread MetricsThread(MetricsRunThread, std::ref(metrics));
    MetricsThread.detach();
  });


  py::class_<avoidmetrics::Mission, std::shared_ptr<avoidmetrics::Mission>>(m, "AvoidMission")
  .def(py::init<>())
  .def(py::init<const std::string&, const avoidlib::mission_parameter&, const int&>())
  .def_readwrite("end_point", &avoidmetrics::Mission::end_point)
  .def("getTrajectory", &avoidmetrics::Mission::getTrajectory)
  .def("CollisionCount", &avoidmetrics::Mission::CollisionCount)
  .def_readwrite("finished", &avoidmetrics::Mission::finished)
  .def_readwrite("stop_flag", &avoidmetrics::Mission::stop_flag)
  .def_readwrite("cal_time", &avoidmetrics::Mission::cal_time)
  .def_readwrite("trials", &avoidmetrics::Mission::trials);

  py::class_<avoidmetrics::Metrics_msg>(m, "MetricsMsg")
  .def(py::init<>())
  .def_readwrite("traversability", &avoidmetrics::Metrics_msg::traversability)
  .def_readwrite("optimality_factor", &avoidmetrics::Metrics_msg::optimality_factor)
  .def_readwrite("average_goal_velocity", &avoidmetrics::Metrics_msg::average_goal_velocity)
  .def_readwrite("mission_progress", &avoidmetrics::Metrics_msg::mission_progress)
  .def_readwrite("relative_gap_size", &avoidmetrics::Metrics_msg::relative_gap_size)
  .def_readwrite("processing_time", &avoidmetrics::Metrics_msg::processing_time)
  .def_readwrite("collision_number", &avoidmetrics::Metrics_msg::collision_number);
}