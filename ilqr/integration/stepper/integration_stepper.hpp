#ifndef ILQR_INTERGRATION_STEPPER_INTEGRATION_STEPPER_HPP
#define ILQR_INTERGRATION_STEPPER_INTEGRATION_STEPPER_HPP

#include <memory>

#include "ilqr/systems/continuous_time/continuous_system.hpp"

namespace ilqr {
namespace integration {
namespace stepper {

using ilqr::systems::continuous_time::ContinuousSystem;

template <std::size_t N, std::size_t M>
class IntegrationStepper {
   public:
    using SystemPtr = std::shared_ptr<ContinuousSystem<N, M>>;
    using State     = Eigen::Matrix<double, N, 1>;
    using Control   = Eigen::Matrix<double, M, 1>;
    using Time      = double;

    constexpr std::size_t GetNumState() const { return N; }

    constexpr std::size_t GetNumControl() const { return M; }

    virtual void Step(const SystemPtr& system, const State& state,
                      const Control& control, const Time& time,
                      const Time& delta_time, State& result) const = 0;
};

}  // namespace stepper
}  // namespace integration
}  // namespace ilqr

#endif  // ILQR_INTERGRATION_STEPPER_INTEGRATION_STEPPER_HPP