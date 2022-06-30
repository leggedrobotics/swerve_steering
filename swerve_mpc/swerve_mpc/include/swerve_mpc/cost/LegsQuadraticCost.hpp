#pragma once

#include <ocs2_core/cost/QuadraticStateCost.h>
#include <swerve_mpc/definitions.hpp>

namespace ocs2{
namespace swerve{

class LegsQuadraticCost final : public QuadraticStateCost {
 public:
  explicit LegsQuadraticCost(matrix_t Q) : QuadraticStateCost(std::move(Q)) {}
  ~LegsQuadraticCost() override = default;

  LegsQuadraticCost* clone() const override { return new LegsQuadraticCost(*this);}
  vector_t getStateDeviation(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories) const{
    vector_t result = vector_t::Zero(1);
    result << state(4);
    result = result - targetTrajectories.getDesiredState(time).tail(1);
    std::cout << "state leg: " << state(3) << std::endl;
    return result;
  }

};

} // namespace swerve
} // namespace ocs2
