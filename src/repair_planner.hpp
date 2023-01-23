#include "ompl/geometric/planners/experience/LightningRetrieveRepair.h"

namespace og = ompl::geometric;
namespace ob = ompl::base;

class LightningRetrieveRepairWrap : public og::LightningRetrieveRepair
{
 public:
  using og::LightningRetrieveRepair::LightningRetrieveRepair;
  ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc,
                          const og::PathGeometricPtr trajectory_heuristic);
};
