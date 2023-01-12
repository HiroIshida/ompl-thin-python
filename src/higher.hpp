#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
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

template <bool Constrained>
std::vector<double> state_to_vec(const ob::State* state, size_t dim)
{
  const ob::RealVectorStateSpace::StateType* rs;
  if constexpr (Constrained) {
    rs = state->as<ompl::base::ConstrainedStateSpace::StateType>()
             ->getState()
             ->as<ob::RealVectorStateSpace::StateType>();
  } else {
    rs = state->as<ob::RealVectorStateSpace::StateType>();
  }
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

template <bool Constrained>
struct CollisionAwareSpaceInformation {
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
    const auto vec = state_to_vec<Constrained>(state, dim);
    this->is_valid_call_count_++;
    return is_valid_(vec);
  }

  ob::SpaceInformationPtr si_;
  std::function<bool(std::vector<double>)> is_valid_;
  size_t is_valid_call_count_;
  const size_t max_is_valid_call_;
};

struct UnconstrianedCollisoinAwareSpaceInformation : public CollisionAwareSpaceInformation<false> {
  UnconstrianedCollisoinAwareSpaceInformation(
      const std::vector<double>& lb,
      const std::vector<double>& ub,
      const std::function<bool(std::vector<double>)>& is_valid,
      size_t max_is_valid_call,
      const std::vector<double>& box_width)
      : CollisionAwareSpaceInformation<false>{nullptr, is_valid, 0, max_is_valid_call}
  {
    const auto space = bound2space(lb, ub);
    si_ = std::make_shared<ob::SpaceInformation>(space);
    if (box_width.size() != space->getDimension()) {
      throw std::runtime_error("box dimension and space dimension mismatch");
    }
    si_->setMotionValidator(std::make_shared<BoxMotionValidator>(si_, box_width));
    si_->setup();
  }
};

struct ConstrainedCollisoinAwareSpaceInformation : public CollisionAwareSpaceInformation<true> {
  ConstrainedCollisoinAwareSpaceInformation(
      const std::shared_ptr<ob::Constraint> constraint,
      const std::vector<double>& lb,
      const std::vector<double>& ub,
      const std::function<bool(std::vector<double>)>& is_valid,
      size_t max_is_valid_call,
      const std::vector<double>& box_width)
      : CollisionAwareSpaceInformation<true>{nullptr, is_valid, 0, max_is_valid_call}
  {
    const auto space = bound2space(lb, ub);
    const auto css = std::make_shared<ob::ProjectedStateSpace>(space, constraint);
    const auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
    si_ = std::static_pointer_cast<ob::SpaceInformation>(csi);
    si_->setup();
  }
};

template <bool Constrained>
struct PlannerBase {
  std::optional<std::vector<std::vector<double>>> solve(const std::vector<double>& start,
                                                        const std::vector<double>& goal,
                                                        bool simplify)
  {
    setup_->clear();
    csi_->resetCount();

    // args shold be eigen maybe?
    Eigen::VectorXd vec_start = Eigen::Map<const Eigen::VectorXd>(&start[0], start.size());
    Eigen::VectorXd vec_goal = Eigen::Map<const Eigen::VectorXd>(&goal[0], goal.size());

    ob::ScopedState<> sstart(csi_->si_->getStateSpace());
    ob::ScopedState<> sgoal(csi_->si_->getStateSpace());

    sstart->as<ob::ConstrainedStateSpace::StateType>()->copy(vec_start);
    sgoal->as<ob::ConstrainedStateSpace::StateType>()->copy(vec_goal);

    setup_->setStartAndGoalStates(sstart, sgoal);

    std::function<bool()> fn = [this]() { return csi_->is_terminatable(); };
    const auto result = setup_->solve(fn);
    if (not result) {
      return {};
    }
    if (result == ob::PlannerStatus::APPROXIMATE_SOLUTION) {
      OMPL_INFORM("reporeted to be solved. But reject it because it'S approx solution");
      return {};
    }
    if (simplify) {
      setup_->simplifySolution(fn);
    }
    const auto p = setup_->getSolutionPath().as<og::PathGeometric>();
    const auto& states = p->getStates();
    auto trajectory = std::vector<std::vector<double>>();
    const size_t dim = start.size();
    for (const auto& state : states) {
      trajectory.push_back(state_to_vec<Constrained>(state, dim));
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

  typedef typename std::conditional<Constrained,
                                    ConstrainedCollisoinAwareSpaceInformation,
                                    UnconstrianedCollisoinAwareSpaceInformation>::type CsiType;
  std::unique_ptr<CsiType> csi_;
  std::unique_ptr<og::SimpleSetup> setup_;
};

struct ConstrainedPlanner : public PlannerBase<true> {
  ConstrainedPlanner(const std::shared_ptr<ob::Constraint> constraint,
                     const std::vector<double>& lb,
                     const std::vector<double>& ub,
                     const std::function<bool(std::vector<double>)>& is_valid,
                     size_t max_is_valid_call,
                     const std::vector<double>& box_width)
      : PlannerBase<true>{nullptr, nullptr}
  {
    csi_ = std::make_unique<ConstrainedCollisoinAwareSpaceInformation>(
        constraint, lb, ub, is_valid, max_is_valid_call, box_width);

    setup_ = std::make_unique<og::SimpleSetup>(csi_->si_);
    setup_->setStateValidityChecker([this](const ob::State* s) { return this->csi_->is_valid(s); });
  }
};

struct UnconstrainedPlannerBase : public PlannerBase<false> {
  UnconstrainedPlannerBase(const std::vector<double>& lb,
                           const std::vector<double>& ub,
                           const std::function<bool(std::vector<double>)>& is_valid,
                           size_t max_is_valid_call,
                           const std::vector<double>& box_width)
      : PlannerBase<false>{nullptr, nullptr}
  {
    csi_ = std::make_unique<UnconstrianedCollisoinAwareSpaceInformation>(
        lb, ub, is_valid, max_is_valid_call, box_width);
    setup_ = std::make_unique<og::SimpleSetup>(csi_->si_);
    setup_->setStateValidityChecker([this](const ob::State* s) { return this->csi_->is_valid(s); });
  }
};

struct OMPLPlanner : public UnconstrainedPlannerBase {
  OMPLPlanner(const std::vector<double>& lb,
              const std::vector<double>& ub,
              const std::function<bool(std::vector<double>)>& is_valid,
              size_t max_is_valid_call,
              const std::vector<double>& box_width,
              const std::string& algo_name)
      : UnconstrainedPlannerBase(lb, ub, is_valid, max_is_valid_call, box_width)
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
        path.push_back(state_to_vec<false>(vert.getState(), dim));
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

struct LightningPlanner : public UnconstrainedPlannerBase {
  LightningPlanner(const LightningDBWrap& dbwrap,
                   const std::vector<double>& lb,
                   const std::vector<double>& ub,
                   const std::function<bool(std::vector<double>)>& is_valid,
                   size_t max_is_valid_call,
                   const std::vector<double>& box_width,
                   const std::string& algo_name)
      : UnconstrainedPlannerBase(lb, ub, is_valid, max_is_valid_call, box_width)
  {
    auto repair_planner = std::make_shared<og::LightningRetrieveRepair>(csi_->si_, dbwrap.db);
    setup_->setPlanner(repair_planner);
  }
};

void setGlobalSeed(size_t seed) { ompl::RNG::setSeed(seed); }
