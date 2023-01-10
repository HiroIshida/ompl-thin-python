#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
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
#include <ompl/tools/lightning/LightningDB.h>
#include <ompl/util/PPM.h>
#include <ompl/util/Time.h>

#include <boost/filesystem.hpp>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "ompl/base/DiscreteMotionValidator.h"
#include "ompl/base/MotionValidator.h"
#include "ompl/base/Planner.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerStatus.h"
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

std::vector<double> state_to_vec(const ob::State* state, size_t dim)
{
  const auto rs = state->as<ob::RealVectorStateSpace::StateType>();
  std::vector<double> vec(dim);
  for (size_t i = 0; i < dim; ++i) {
    vec[i] = rs->values[i];
  }
  return vec;
};

og::PathGeometric points_to_pathgeometric(const std::vector<std::vector<double>>& points,
                                          ob::SpaceInformationPtr si)
{
  auto pg = og::PathGeometric(si);
  for (const auto& point : points) {
    ob::State* s = si->getStateSpace()->allocState();
    auto rs = s->as<ob::RealVectorStateSpace::StateType>();
    for (size_t i = 0; i < si->getStateDimension(); ++i) {
      rs->values[i] = point.at(i);
    }
    pg.append(rs);
  }
  return pg;
}

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

class BoxMotionValidator : public ob::MotionValidator
{
 public:
  BoxMotionValidator(const ob::SpaceInformationPtr& si, std::vector<double> width)
      : ob::MotionValidator(si), width_(width)
  {
  }

  bool checkMotion(const ob::State* s1, const ob::State* s2) const
  {
    const auto rs1 = s1->as<ob::RealVectorStateSpace::StateType>();
    const auto rs2 = s2->as<ob::RealVectorStateSpace::StateType>();

    // find longest (relative) axis index
    double diff_longest_axis;
    double max_diff = -std::numeric_limits<double>::infinity();
    size_t longest_idx = 0;
    for (size_t idx = 0; idx < si_->getStateDimension(); ++idx) {
      const double check_width = width_[idx];
      const double diff = rs2->values[idx] - rs1->values[idx];
      const double abs_scaled_diff = std::abs(diff) / check_width;
      if (abs_scaled_diff > max_diff) {
        max_diff = abs_scaled_diff;
        longest_idx = idx;
        diff_longest_axis = diff;
      }
    }
    if (std::abs(diff_longest_axis) < 1e-6) {
      return true;
    }

    // main
    const auto s_test = si_->allocState()->as<ob::RealVectorStateSpace::StateType>();

    const auto space = si_->getStateSpace();
    const double step_ratio = width_[longest_idx] / std::abs(diff_longest_axis);

    double travel_rate = 0.0;
    while (travel_rate + step_ratio < 1.0) {
      travel_rate += step_ratio;
      space->interpolate(rs1, rs2, travel_rate, s_test);
      if (!si_->isValid(s_test)) {
        return false;
      }
    }
    return (si_->isValid(rs2));
  }

  bool checkMotion(const ob::State* s1,
                   const ob::State* s2,
                   std::pair<ob::State*, double>& lastValid) const
  {
    return checkMotion(s1, s2);
  }

 private:
  std::vector<double> width_;
};

struct CollisionAwareSpaceInformation {
  CollisionAwareSpaceInformation(const std::vector<double>& lb,
                                 const std::vector<double>& ub,
                                 const std::function<bool(std::vector<double>)>& is_valid,
                                 size_t max_is_valid_call,
                                 const std::vector<double>& box_width)
      : CollisionAwareSpaceInformation(bound2space(lb, ub), is_valid, max_is_valid_call, box_width)
  {
  }

  CollisionAwareSpaceInformation(const ob::StateSpacePtr space,
                                 const std::function<bool(std::vector<double>)>& is_valid,
                                 size_t max_is_valid_call,
                                 const std::vector<double>& box_width)
      : is_valid_(is_valid), is_valid_call_count_(0), max_is_valid_call_(max_is_valid_call)
  {
    si_ = std::make_shared<ob::SpaceInformation>(space);
    if (box_width.size() != space->getDimension()) {
      throw std::runtime_error("box dimension and space dimension mismatch");
    }
    si_->setMotionValidator(std::make_shared<BoxMotionValidator>(si_, box_width));
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
    const auto vec = state_to_vec(state, dim);
    this->is_valid_call_count_++;
    return is_valid_(vec);
  }

  ob::SpaceInformationPtr si_;
  std::function<bool(std::vector<double>)> is_valid_;
  size_t is_valid_call_count_;
  const size_t max_is_valid_call_;
};

class PlannerBase
{
 public:
  PlannerBase(const std::vector<double>& lb,
              const std::vector<double>& ub,
              const std::function<bool(std::vector<double>)>& is_valid,
              size_t max_is_valid_call,
              const std::vector<double>& box_width)
      : csi_(nullptr), setup_(nullptr)
  {
    csi_ = std::make_unique<CollisionAwareSpaceInformation>(
        lb, ub, is_valid, max_is_valid_call, box_width);
    setup_ = std::make_unique<og::SimpleSetup>(csi_->si_);
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
    if (result == ob::PlannerStatus::APPROXIMATE_SOLUTION) {
      OMPL_INFORM("reporeted to be solved. But reject it because it'S approx solution");
      return {};
    }
    const auto p = setup_->getSolutionPath().as<og::PathGeometric>();
    const auto& states = p->getStates();
    auto trajectory = std::vector<std::vector<double>>();
    for (const auto& state : states) {
      trajectory.push_back(state_to_vec(state, dim));
    }
    return trajectory;
  }

  void resetIsValid(const std::function<bool(std::vector<double>)>& is_valid)
  {
    csi_->is_valid_ = is_valid;
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
  std::unique_ptr<og::SimpleSetup> setup_;
};

struct OMPLPlanner : public PlannerBase {
  OMPLPlanner(const std::vector<double>& lb,
              const std::vector<double>& ub,
              const std::function<bool(std::vector<double>)>& is_valid,
              size_t max_is_valid_call,
              const std::vector<double>& box_width,
              const std::string& algo_name)
      : PlannerBase(lb, ub, is_valid, max_is_valid_call, box_width)
  {
    const auto algo = create_algorithm(algo_name);
    setup_->setPlanner(algo);
  }
};

struct LightningDBWrap {
  LightningDBWrap(size_t n_dim)
  {
    auto space = ob::StateSpacePtr(new ob::RealVectorStateSpace(n_dim));
    si = std::make_shared<ob::SpaceInformation>(space);
    db = std::make_shared<ot::LightningDB>(space);
  }

  void addExperience(const std::vector<std::vector<double>>& points)
  {
    auto pg = points_to_pathgeometric(points, si);
    double insertion_time;
    db->addPath(pg, insertion_time);
  }

  std::vector<std::vector<std::vector<double>>> getExperiencedPaths()
  {
    const auto dim = si->getStateSpace()->getDimension();

    std::vector<std::vector<std::vector<double>>> paths;

    std::vector<ob::PlannerDataPtr> datas;
    db->getAllPlannerDatas(datas);
    for (const auto& data : datas) {
      std::vector<std::vector<double>> path;
      for (std::size_t i = 0; i < data->numVertices(); ++i) {
        const auto vert = data->getVertex(i);
        path.push_back(state_to_vec(vert.getState(), dim));
      }
      paths.push_back(path);
    }
    return paths;
  }

  size_t getExperiencesCount() { return db->getExperiencesCount(); }

  void save(const std::string& fileName) { db->save(fileName); }

  void load(const std::string& fileName) { db->load(fileName); }

  ob::SpaceInformationPtr si;
  ot::LightningDBPtr db;
};

struct LightningPlanner : public PlannerBase {
  LightningPlanner(const LightningDBWrap& dbwrap,
                   const std::vector<double>& lb,
                   const std::vector<double>& ub,
                   const std::function<bool(std::vector<double>)>& is_valid,
                   size_t max_is_valid_call,
                   const std::vector<double>& box_width,
                   const std::string& algo_name)
      : PlannerBase(lb, ub, is_valid, max_is_valid_call, box_width)
  {
    auto repair_planner = std::make_shared<og::LightningRetrieveRepair>(csi_->si_, dbwrap.db);
    setup_->setPlanner(repair_planner);
  }
};

struct PathSimplifierWrapper {
  PathSimplifierWrapper(const std::vector<double>& lb,
                        const std::vector<double>& ub,
                        const std::function<bool(std::vector<double>)>& is_valid,
                        size_t max_is_valid_call,
                        const std::vector<double>& box_width)
  {
    csi_ = std::make_unique<CollisionAwareSpaceInformation>(
        lb, ub, is_valid, max_is_valid_call, box_width);
    psk_ = std::make_shared<og::PathSimplifier>(csi_->si_);
    csi_->si_->setStateValidityChecker(
        [this](const ob::State* s) { return this->csi_->is_valid(s); });
  }

  std::vector<std::vector<double>> simplify(const std::vector<std::vector<double>>& points)
  {
    ompl::time::point simplifyStart = ompl::time::now();
    csi_->resetCount();
    const size_t dim = csi_->si_->getStateDimension();
    auto pg = points_to_pathgeometric(points, csi_->si_);
    std::function<bool()> fn = [this]() { return csi_->is_terminatable(); };
    psk_->simplify(pg, fn);
    std::vector<std::vector<double>> points_out;
    for (const auto state : pg.getStates()) {
      points_out.push_back(state_to_vec(state, dim));
    }
    double simplifyTime = ompl::time::seconds(ompl::time::now() - simplifyStart);
    OMPL_INFORM("ompl_thin: Path simplification took %f seconds", simplifyTime);
    return points_out;
  }

  std::unique_ptr<CollisionAwareSpaceInformation> csi_;
  og::PathSimplifierPtr psk_;
};
