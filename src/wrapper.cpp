#include <ompl/geometric/PathGeometric.h>
#include <ompl/tools/lightning/LightningDB.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>
#include <vector>

#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/util/RandomNumbers.h"
#include "ompl_higher.hpp"

namespace py = pybind11;

void setGlobalSeed(size_t seed) { ompl::RNG::setSeed(seed); }

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;


PYBIND11_MODULE(_omplpy, m)
{
  m.doc() = "unofficial ompl python wrapper";
  m.def("set_random_seed", &setGlobalSeed);
  py::class_<OMPLPlanner>(m, "_OMPLPlanner")
      .def(py::init<std::vector<double>,
                    std::vector<double>,
                    std::function<bool(std::vector<double>)>,
                    size_t,
                    std::vector<double>,
                    std::string>())
      .def("reset_is_valid", &OMPLPlanner::resetIsValid)
      .def("solve", &OMPLPlanner::solve);

  py::class_<LightningDBWrap>(m, "_LightningDB")
      .def(py::init<size_t>())
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
                    std::string>())
      .def("reset_is_valid", &LightningPlanner::resetIsValid)
      .def("solve", &LightningPlanner::solve);
}
