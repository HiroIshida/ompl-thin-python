#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/tools/experience/ExperienceSetup.h>
#include <ompl/tools/lightning/Lightning.h>
#include <ompl/util/PPM.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <boost/filesystem.hpp>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include "ompl/base/Planner.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace py = pybind11;

template <typename SetupT>
struct SetupWrapper {
  SetupWrapper(std::vector<double> lb,
               std::vector<double> ub,
               std::function<bool(std::vector<double>)> is_valid,
               size_t max_is_valid_call)
      : SetupWrapper(bound2space(lb, ub), is_valid, max_is_valid_call)
  {
  }

  SetupWrapper(ob::StateSpacePtr space,
               std::function<bool(std::vector<double>)> is_valid,
               size_t max_is_valid_call)
      : setup_(nullptr),
        is_valid_(is_valid),
        is_valid_call_count_(0),
        max_is_valid_call_(max_is_valid_call)
  {
    setup_ = std::make_unique<SetupT>(space);
    setup_->setStateValidityChecker(
        [this](const ob::State* s) { return this->is_valid(s); });
  }

  void resetCount() { this->is_valid_call_count_ = 0; }

  static std::shared_ptr<ob::StateSpace> bound2space(
      const std::vector<double>& lb, const std::vector<double>& ub)
  {
    const size_t dim = lb.size();
    auto bounds = ob::RealVectorBounds(dim);
    bounds.low = lb;
    bounds.high = ub;
    const auto space(std::make_shared<ob::RealVectorStateSpace>(dim));
    space->setBounds(bounds);
    space->setup();
    return space;
  }

  bool is_terminatable() const
  {
    return is_valid_call_count_ > max_is_valid_call_;
  }

  bool is_valid(const ob::State* state)
  {
    const size_t dim = this->setup_->getSpaceInformation()->getStateDimension();
    auto vec = std::vector<double>(dim);
    const auto& rs = state->as<ob::RealVectorStateSpace::StateType>();
    for (size_t i = 0; i < vec.size(); ++i) {
      vec[i] = rs->values[i];
    }
    this->is_valid_call_count_++;
    return is_valid_(vec);
  }

  std::unique_ptr<SetupT> setup_;
  std::function<bool(std::vector<double>)> is_valid_;
  size_t is_valid_call_count_;
  const size_t max_is_valid_call_;
};

void setAlgorithm(const SetupWrapper<og::SimpleSetup>& sw,
                  const std::string& algoname)
{
  // don't use enum for algorithm. Enum seem to cause memory-leak by pybind
  if (algoname.compare("rrtconnect") == 0) {
    const auto space_info = sw.setup_->getSpaceInformation();
    sw.setup_->setPlanner(std::make_shared<og::RRTConnect>(space_info));
  } else {
    throw std::runtime_error("not supported");
  }
}

std::optional<std::vector<std::vector<double>>> solve(
    const SetupWrapper<og::SimpleSetup>& sw,
    const std::vector<double>& start,
    const std::vector<double>& goal)
{
  const auto space = sw.setup_->getStateSpace();
  ob::ScopedState<ob::RealVectorStateSpace> start_state(space),
      goal_state(space);

  const size_t dim = start.size();

  for (size_t i = 0; i < dim; ++i) {
    start_state->values[i] = start[i];
    goal_state->values[i] = goal[i];
  }
  sw.setup_->setStartAndGoalStates(start_state, goal_state);

  std::function<bool()> fn = [&sw]() { return sw.is_terminatable(); };
  const auto result = sw.setup_->solve(fn);
  if (not result) {
    return {};
  }
  const auto p = sw.setup_->getSolutionPath().as<og::PathGeometric>();
  const auto& states = p->getStates();
  auto trajectory =
      std::vector<std::vector<double>>(states.size(), std::vector<double>(dim));
  for (size_t i = 0; i < states.size(); ++i) {
    const auto& rs = states[i]->as<ob::RealVectorStateSpace::StateType>();
    for (size_t j = 0; j < dim; ++j) {
      trajectory[i][j] = rs->values[j];
    }
  }
  return trajectory;
}

PYBIND11_MODULE(_omplpy, m)
{
  m.doc() = "unofficial ompl python wrapper";
  py::class_<SetupWrapper<og::SimpleSetup>>(m, "SimpleSetup")
      .def(py::init<std::vector<double>,
                    std::vector<double>,
                    std::function<bool(std::vector<double>)>,
                    size_t>());
  m.def("set_algorithm", setAlgorithm, "set aglrithm");
  m.def("solve", solve, "solve");
}
