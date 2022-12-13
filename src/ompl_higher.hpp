#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
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
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/StateValidityChecker.h"

#define STRING(str) #str
#define ALLOCATE_ALGO(ALGO)                                   \
  if (name.compare(#ALGO) == 0) {                             \
    const auto algo = std::make_shared<og::ALGO>(space_info); \
    return std::static_pointer_cast<ob::Planner>(algo);       \
  }

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

struct CollisionAwareSpaceInformation {
  CollisionAwareSpaceInformation(std::vector<double> lb,
                                 std::vector<double> ub,
                                 std::function<bool(std::vector<double>)> is_valid,
                                 size_t max_is_valid_call,
                                 double fraction)
      : CollisionAwareSpaceInformation(bound2space(lb, ub), is_valid, max_is_valid_call, fraction)
  {
  }

  CollisionAwareSpaceInformation(ob::StateSpacePtr space,
                                 std::function<bool(std::vector<double>)> is_valid,
                                 size_t max_is_valid_call,
                                 double fraction)
      : is_valid_(is_valid), is_valid_call_count_(0), max_is_valid_call_(max_is_valid_call)
  {
    si_ = std::make_shared<ob::SpaceInformation>(space);
    si_->getStateSpace()->setLongestValidSegmentFraction(fraction);
    si_->setup();
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
    const size_t dim = si_->getStateDimension();
    auto vec = std::vector<double>(dim);
    const auto& rs = state->as<ob::RealVectorStateSpace::StateType>();
    for (size_t i = 0; i < vec.size(); ++i) {
      vec[i] = rs->values[i];
    }
    this->is_valid_call_count_++;

    return is_valid_(vec);
  }

  ob::SpaceInformationPtr si_;
  std::function<bool(std::vector<double>)> is_valid_;
  size_t is_valid_call_count_;
  const size_t max_is_valid_call_;
};

template <typename SetupT,
          typename std::enable_if<std::is_base_of<og::SimpleSetup, SetupT>::value, int>::type = 0>
class PlannerBase
{
 public:
  PlannerBase(std::vector<double> lb,
              std::vector<double> ub,
              std::function<bool(std::vector<double>)> is_valid,
              size_t max_is_valid_call,
              double interval)
      : csi_(nullptr), setup_(nullptr)
  {
    csi_ = std::make_unique<CollisionAwareSpaceInformation>(
        lb, ub, is_valid, max_is_valid_call, interval);
    setup_ = std::make_unique<SetupT>(csi_->si_);
    setup_->setStateValidityChecker([this](const ob::State* s) { return this->csi_->is_valid(s); });
  }

  std::optional<std::vector<std::vector<double>>> solve(const std::vector<double>& start,
                                                        const std::vector<double>& goal)
  {
    setup_->clear();
    csi_->resetCount();
    const auto space = setup_->getStateSpace();
    ob::ScopedState<ob::RealVectorStateSpace> start_state(space), goal_state(space);

    const size_t dim = start.size();

    for (size_t i = 0; i < dim; ++i) {
      start_state->values[i] = start[i];
      goal_state->values[i] = goal[i];
    }
    setup_->setStartAndGoalStates(start_state, goal_state);

    std::function<bool()> fn = [this]() { return csi_->is_terminatable(); };
    const auto result = setup_->solve(fn);
    if (not result) {
      return {};
    }
    const auto p = setup_->getSolutionPath().template as<og::PathGeometric>();
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

  std::shared_ptr<ob::Planner> create_algorithm(const std::string& name)
  {
    const auto space_info = csi_->si_;
    std::shared_ptr<ob::Planner> algo_out;
    // informed
    ALLOCATE_ALGO(ABITstar);
    ALLOCATE_ALGO(AITstar);
    ALLOCATE_ALGO(BITstar);
    // kpiece
    ALLOCATE_ALGO(BKPIECE1);
    ALLOCATE_ALGO(KPIECE1);
    ALLOCATE_ALGO(LBKPIECE1);
    // rrt
    ALLOCATE_ALGO(RRTConnect);
    ALLOCATE_ALGO(RRT);
    ALLOCATE_ALGO(RRTstar);
    ALLOCATE_ALGO(SORRTstar);
    ALLOCATE_ALGO(RRTsharp);
    ALLOCATE_ALGO(InformedRRTstar);
    // rpm
    ALLOCATE_ALGO(PRMstar);
    ALLOCATE_ALGO(LazyPRMstar);
    ALLOCATE_ALGO(LazyPRM);
    ALLOCATE_ALGO(PRM);
    // fmt
    ALLOCATE_ALGO(BFMT);
    ALLOCATE_ALGO(FMT);
    throw std::runtime_error("algorithm " + name + " is not supported");
  }

  std::unique_ptr<CollisionAwareSpaceInformation> csi_;
  std::unique_ptr<SetupT> setup_;
};

struct OMPLPlanner : public PlannerBase<og::SimpleSetup> {
  OMPLPlanner(std::vector<double> lb,
              std::vector<double> ub,
              std::function<bool(std::vector<double>)> is_valid,
              size_t max_is_valid_call,
              double interval,
              std::string algo_name)
      : PlannerBase(lb, ub, is_valid, max_is_valid_call, interval)
  {
    const auto algo = create_algorithm(algo_name);
    setup_->setPlanner(algo);
  }
};

struct LightningPlanner : public PlannerBase<ot::Lightning> {
  LightningPlanner(std::vector<double> lb,
                   std::vector<double> ub,
                   std::function<bool(std::vector<double>)> is_valid,
                   size_t max_is_valid_call,
                   double interval,
                   std::string algo_name)
      : PlannerBase(lb, ub, is_valid, max_is_valid_call, interval)
  {
    const auto algo = create_algorithm(algo_name);
    setup_->setPlanner(algo);
    setup_->setRepairPlanner(algo);
    scratchMode();
  }
  std::optional<std::vector<std::vector<double>>> solve(const std::vector<double>& start,
                                                        const std::vector<double>& goal)
  {
    const auto ret = PlannerBase<ot::Lightning>::solve(start, goal);
    if (recall_mode_) {
      recall_called_ = true;
    }
    return ret;
  }

  void recallMode()
  {
    std::vector<ob::PlannerDataPtr> datas;
    setup_->getAllPlannerDatas(datas);
    if (datas.size() < 1) {
      throw std::runtime_error("recall mode can be enabled only if planner have experiences");
    }

    setup_->enablePlanningFromScratch(false);
    setup_->enablePlanningFromRecall(true);
    recall_mode_ = true;
  }

  void scratchMode()
  {
    setup_->enablePlanningFromScratch(true);
    setup_->enablePlanningFromRecall(false);
    recall_mode_ = false;
  }

  std::optional<size_t> getLatestActivatedIndex()
  {
    if (!recall_called_) {
      // without this, segmentation fault occurs
      return {};
    }
    std::vector<ob::PlannerDataPtr> datas;
    setup_->getAllPlannerDatas(datas);
    const auto& rrplanner = setup_->getLightningRetrieveRepairPlanner();
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
    setup_->setFilePath(file_name);
    const bool success = setup_->save();
    if (!success) {
      throw std::runtime_error("failed to save to " + file_name);
    }
    setup_->setFilePath("");  // I don't want to have filepath as internal state
  }

  void loadExperience(const std::string& file_name)
  {
    const size_t exp_count = setup_->getExperiencesCount();
    if (exp_count > 0) {
      const std::string message =
          "cannot load because you have " + std::to_string(exp_count) + " experiences";
      throw std::runtime_error(message);
    }
    setup_->setFilePath(file_name);
    setup_->setup();          // load database
    setup_->setFilePath("");  // I don't want to have filepath as internal state
  }

  std::vector<std::vector<std::vector<double>>> getExperiencedPaths()
  {
    const auto dim = setup_->getStateSpace()->getDimension();

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
    setup_->getAllPlannerDatas(datas);
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

  /** \brief indicate that currently the planner is in recall mode */
  bool recall_mode_{false};

  /** \brief indicate solve called with recall mode. this indicates caches are save in recalling */
  bool recall_called_{false};
};
