#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "higher.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_omplpy, m)
{
  m.doc() = "unofficial ompl python wrapper";
  m.def("set_random_seed", &setGlobalSeed);

  py::class_<ConstrainedPlanner>(m, "_ConstrainedPlanner")
      .def(py::init<const ConstFn&,
                    const ConstJacFn&,
                    std::vector<double>,
                    std::vector<double>,
                    std::function<bool(std::vector<double>)>,
                    size_t,
                    std::vector<double>,
                    std::string,
                    std::optional<double>>())
      .def("reset_is_valid", &ConstrainedPlanner::resetIsValid)
      .def("solve", &ConstrainedPlanner::solve);

  py::class_<OMPLPlanner>(m, "_OMPLPlanner")
      .def(py::init<std::vector<double>,
                    std::vector<double>,
                    std::function<bool(std::vector<double>)>,
                    size_t,
                    std::vector<double>,
                    std::string,
                    std::optional<double>>())
      .def("reset_is_valid", &OMPLPlanner::resetIsValid)
      .def("solve", &OMPLPlanner::solve);

  py::class_<LightningDBWrap>(m, "_LightningDB")
      .def(py::init<size_t>())
      .def("save", &LightningDBWrap::save)
      .def("load", &LightningDBWrap::load)
      .def("add_experience", &LightningDBWrap::addExperience)
      .def("get_experienced_paths", &LightningDBWrap::getExperiencedPaths)
      .def("get_experiences_count", &LightningDBWrap::getExperiencesCount);

  py::class_<LightningPlanner>(m, "_LightningPlanner")
      .def(py::init<LightningDBWrap,
                    std::vector<double>,
                    std::vector<double>,
                    std::function<bool(std::vector<double>)>,
                    size_t,
                    std::vector<double>,
                    std::string,
                    std::optional<double>>())
      .def("reset_is_valid", &LightningPlanner::resetIsValid)
      .def("solve", &LightningPlanner::solve);

  py::class_<LightningRepairPlanner>(m, "_LightningRepairPlanner")
      .def(py::init<std::vector<double>,
                    std::vector<double>,
                    std::function<bool(std::vector<double>)>,
                    size_t,
                    std::vector<double>,
                    std::string,
                    std::optional<double>>())
      .def("reset_is_valid", &LightningRepairPlanner::resetIsValid)
      .def("solve", &LightningRepairPlanner::solve)
      .def("set_heuristic", &LightningRepairPlanner::set_heuristic);
}
