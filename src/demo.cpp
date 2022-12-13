#include <vector>

#include "ompl_higher.hpp"

int main()
{
  const auto lb = std::vector<double>{0.0, 0.0};
  const auto ub = std::vector<double>{1.0, 1.0};
  const auto is_valid = [](const std::vector<double>& vec) { return true; };
  const size_t max_is_valid_call = 1000;
  const std::vector<double> box{0.3};

  auto planner = LightningPlanner(lb, ub, is_valid, max_is_valid_call, box, "RRTConnect");

  // get experience
  const auto start = std::vector<double>{0.1, 0.1};
  const auto goal = std::vector<double>{0.9, 0.9};
  planner.solve(start, goal);

  // use previous experience to solve faster
  planner.recallMode();
  planner.solve(start, goal);
}
