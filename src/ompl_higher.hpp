#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/tools/experience/ExperienceSetup.h>
#include <ompl/tools/lightning/Lightning.h>
#include <ompl/util/PPM.h>

#include <boost/filesystem.hpp>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "ompl/base/DiscreteMotionValidator.h"
#include "ompl/base/MotionValidator.h"
#include "ompl/base/Planner.h"
#include "ompl/base/PlannerData.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

class AllPassMotionValidator : public ob::MotionValidator
{
 public:
  using ob::MotionValidator::MotionValidator;
  bool checkMotion(const ob::State* s1, const ob::State* s2) const { return true; }

  bool checkMotion(const ob::State* s1,
                   const ob::State* s2,
                   std::pair<ob::State*, double>& lastValid) const
  {
    return true;
  }
};

template <typename SetupT,
          typename std::enable_if<std::is_base_of<og::SimpleSetup, SetupT>::value, int>::type = 0>
struct SetupWrapper {
  SetupWrapper(std::vector<double> lb,
               std::vector<double> ub,
               std::function<bool(std::vector<double>)> is_valid,
               size_t max_is_valid_call,
               double fraction)
      : SetupWrapper(bound2space(lb, ub), is_valid, max_is_valid_call, fraction)
  {
  }

  SetupWrapper(ob::StateSpacePtr space,
               std::function<bool(std::vector<double>)> is_valid,
               size_t max_is_valid_call,
               double fraction)
      : setup_(nullptr),
        is_valid_(is_valid),
        is_valid_call_count_(0),
        max_is_valid_call_(max_is_valid_call)
  {
    setup_ = std::make_unique<SetupT>(space);

    // TODO: make this configurable from constructor
    const auto si = this->setup_->getSpaceInformation();
    si->getStateSpace()->setLongestValidSegmentFraction(fraction);
    si->setup();
    // si->setMotionValidator(std::make_shared<AllPassMotionValidator>(si));
    setup_->setStateValidityChecker([this](const ob::State* s) { return this->is_valid(s); });
  }

  void resetCount() { this->is_valid_call_count_ = 0; }

  static std::shared_ptr<ob::StateSpace> bound2space(const std::vector<double>& lb,
                                                     const std::vector<double>& ub)
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

  bool is_terminatable() const { return is_valid_call_count_ > max_is_valid_call_; }

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

  std::shared_ptr<ob::Planner> create_algorithm(const std::string& name)
  {
    const auto space_info = this->setup_->getSpaceInformation();
    std::shared_ptr<ob::Planner> algo_out;
    if (name.compare("rrtconnect") == 0) {
      const auto algo = std::make_shared<og::RRTConnect>(space_info);
      algo_out = algo;
      return std::static_pointer_cast<ob::Planner>(algo);
    } else {
      throw std::runtime_error("not supported");
    }
    return algo_out;
  }

  std::unique_ptr<SetupT> setup_;
  std::function<bool(std::vector<double>)> is_valid_;
  size_t is_valid_call_count_;
  const size_t max_is_valid_call_;
};

template <typename SetupT,
          typename std::enable_if<std::is_base_of<og::SimpleSetup, SetupT>::value, int>::type = 0>
std::optional<std::vector<std::vector<double>>> solve(const SetupWrapper<SetupT>& sw,
                                                      const std::vector<double>& start,
                                                      const std::vector<double>& goal)
{
  const auto space = sw.setup_->getStateSpace();
  ob::ScopedState<ob::RealVectorStateSpace> start_state(space), goal_state(space);

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
  const auto p = sw.setup_->getSolutionPath().template as<og::PathGeometric>();
  const auto& states = p->getStates();
  auto trajectory = std::vector<std::vector<double>>(states.size(), std::vector<double>(dim));
  for (size_t i = 0; i < states.size(); ++i) {
    const auto& rs = states[i]->template as<ob::RealVectorStateSpace::StateType>();
    for (size_t j = 0; j < dim; ++j) {
      trajectory[i][j] = rs->values[j];
    }
  }
  return trajectory;
}

struct OMPLPlanner {
  OMPLPlanner(std::vector<double> lb,
              std::vector<double> ub,
              std::function<bool(std::vector<double>)> is_valid,
              size_t max_is_valid_call,
              double interval)
      : sw_(SetupWrapper<og::SimpleSetup>(lb, ub, is_valid, max_is_valid_call, interval))
  {
    const std::string algo_name = "rrtconnect";
    const auto algo = this->sw_.create_algorithm(algo_name);
    this->sw_.setup_->setPlanner(algo);
  }
  std::optional<std::vector<std::vector<double>>> solve(const std::vector<double>& start,
                                                        const std::vector<double>& goal)
  {
    this->sw_.setup_->clear();
    return ::solve(this->sw_, start, goal);
  }

  SetupWrapper<og::SimpleSetup> sw_;
};

struct LightningPlanner {
  LightningPlanner(std::vector<double> lb,
                   std::vector<double> ub,
                   std::function<bool(std::vector<double>)> is_valid,
                   size_t max_is_valid_call,
                   double interval)
      : sw_(SetupWrapper<ot::Lightning>(lb, ub, is_valid, max_is_valid_call, interval))
  {
    const std::string algo_name = "rrtconnect";
    const auto algo = this->sw_.create_algorithm(algo_name);
    this->sw_.setup_->setPlanner(algo);
    this->sw_.setup_->setRepairPlanner(algo);
    scratchMode();
  }
  std::optional<std::vector<std::vector<double>>> solve(const std::vector<double>& start,
                                                        const std::vector<double>& goal)
  {
    this->sw_.setup_->clear();
    const auto ret = ::solve(this->sw_, start, goal);
    if (recall_mode_) {
      recall_called_ = true;
    }
    return ret;
  }

  void recallMode()
  {
    std::vector<ob::PlannerDataPtr> datas;
    sw_.setup_->getAllPlannerDatas(datas);
    if (datas.size() < 1) {
      throw std::runtime_error("recall mode can be enabled only if planner have experiences");
    }

    this->sw_.setup_->enablePlanningFromScratch(false);
    this->sw_.setup_->enablePlanningFromRecall(true);
    this->recall_mode_ = true;
  }

  void scratchMode()
  {
    this->sw_.setup_->enablePlanningFromScratch(true);
    this->sw_.setup_->enablePlanningFromRecall(false);
    this->recall_mode_ = false;
  }

  std::optional<size_t> getLatestActivatedIndex()
  {
    if (!recall_called_) {
      // without this, segmentation fault occurs
      return {};
    }
    std::vector<ob::PlannerDataPtr> datas;
    sw_.setup_->getAllPlannerDatas(datas);
    const auto& rrplanner = sw_.setup_->getLightningRetrieveRepairPlanner();
    const ob::PlannerDataPtr activated = rrplanner.getChosenRecallPath();
    const auto it = std::find(datas.begin(), datas.end(), activated);

    if (it != datas.end()) {
      return it - datas.begin();
    } else {
      return {};
    }
  }

  void dumpExperience(const std::string& file_name)
  {
    sw_.setup_->setFilePath(file_name);
    const bool success = sw_.setup_->save();
    if (!success) {
      throw std::runtime_error("failed to save to " + file_name);
    }
    sw_.setup_->setFilePath("");  // I don't want to have filepath as internal state
  }

  void loadExperience(const std::string& file_name)
  {
    const size_t exp_count = sw_.setup_->getExperiencesCount();
    if (exp_count > 0) {
      const std::string message =
          "cannot load because you have " + std::to_string(exp_count) + " experiences";
      throw std::runtime_error(message);
    }
    sw_.setup_->setFilePath(file_name);
    sw_.setup_->setup();          // load database
    sw_.setup_->setFilePath("");  // I don't want to have filepath as internal state
  }

  std::vector<std::vector<std::vector<double>>> getExperiencedPaths()
  {
    const auto dim = sw_.setup_->getStateSpace()->getDimension();

    const auto state_to_vec = [dim](const ob::State* state) {
      const auto rs = state->as<ob::RealVectorStateSpace::StateType>();
      std::vector<double> vec(dim);
      for (size_t i = 0; i < dim; ++i) {
        vec[i] = rs->values[i];
      }
      return vec;
    };

    std::vector<std::vector<std::vector<double>>> paths;

    std::vector<ob::PlannerDataPtr> datas;
    this->sw_.setup_->getAllPlannerDatas(datas);
    for (const auto& data : datas) {
      std::vector<std::vector<double>> path;
      for (std::size_t i = 0; i < data->numVertices(); ++i) {
        const auto vert = data->getVertex(i);
        path.push_back(state_to_vec(vert.getState()));
      }
      paths.push_back(path);
    }
    return paths;
  }

  SetupWrapper<ot::Lightning> sw_;

  /** \brief indicate that currently the planner is in recall mode */
  bool recall_mode_{false};

  /** \brief indicate solve called with recall mode. this indicates caches are save in recalling */
  bool recall_called_{false};
};
