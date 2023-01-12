#include "higher.hpp"
#include "ompl/base/Constraint.h"
#include <memory>

int main(){
  class SphereConstraint : public ob::Constraint
  {
  public:
      SphereConstraint() : ob::Constraint(3, 1) {}

      void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
      {
          out[0] = x.norm() - 1;
      }

      void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
      {
          out = x.transpose().normalized();
      }
  };

  std::shared_ptr<ob::Constraint> cst = std::make_shared<SphereConstraint>();
  const auto all_pass = [](std::vector<double> vec){return true;};
  const auto box = std::vector<double>{0.1, 0.1, 0.1};

  auto planner = ConstrainedPlanner(cst, std::vector{-2., -2., -2.}, std::vector{2., 2., 2.}, all_pass, 100000, box);
  planner.solve(std::vector{-1., 0., 0.}, std::vector{1., 0., 0.}, false);

  auto simplePath = planner.setup_->getSolutionPath();
  std::cout << "length: " << simplePath.length() << std::endl;
  const auto p = planner.setup_->getSolutionPath().as<og::PathGeometric>();
  const auto& states = p->getStates();
  for (const auto& state : states) {
    std::vector<double> reals;
    const auto ss = state->as<ompl::base::ConstrainedStateSpace::StateType>()->getState()->as<ob::RealVectorStateSpace::StateType>();
    std::cout << ss->values[0] << ", " << ss->values[1] << ", " << ss->values[2] << std::endl;
  }
  
}
