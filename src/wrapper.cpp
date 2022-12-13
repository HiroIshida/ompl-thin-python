#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <vector>

#include "ompl_higher.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_omplpy, m)
{
  m.doc() = "unofficial ompl python wrapper";
  py::class_<OMPLPlanner>(m, "_OMPLPlanner")
      .def(py::init<std::vector<double>,
                    std::vector<double>,
                    std::function<bool(std::vector<double>)>,
                    size_t,
                    double,
                    std::string>())
      .def("solve", &OMPLPlanner::solve);
  py::class_<LightningPlanner>(m, "_LightningPlanner")
      .def(py::init<std::vector<double>,
                    std::vector<double>,
                    std::function<bool(std::vector<double>)>,
                    size_t,
                    double,
                    std::string>())
      .def("solve", &LightningPlanner::solve)
      .def("recall", &LightningPlanner::recallMode)
      .def("scratch", &LightningPlanner::scratchMode)
      .def("get_experienced_paths", &LightningPlanner::getExperiencedPaths)
      .def("dump", &LightningPlanner::dumpExperience)
      .def("load", &LightningPlanner::loadExperience)
      .def("get_latest_activated_index", &LightningPlanner::getLatestActivatedIndex);
}
