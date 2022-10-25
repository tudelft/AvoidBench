#include "rpg_common/sim3.h"

#include <Eigen/Dense>
#include <glog/logging.h>

namespace rpg_common {
namespace sim3 {

Sim3 mean(const std::vector<Sim3>& sim3s)
{
  CHECK(!sim3s.empty());
  Eigen::Matrix<double, 7, 1> sum;
  sum.setZero();
  for (const Sim3& sim3 : sim3s)
  {
    sum += sim3.log();
  }
  sum /= sim3s.size();
  return Sim3(sum);
}

}  // namespace sim3
}  // namespace rpg_common
