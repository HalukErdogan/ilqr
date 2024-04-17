#ifndef ILQR_INTEGRATION_ITERATOR_INTEGRATION_ITERATOR_HPP
#define ILQR_INTEGRATION_ITERATOR_INTEGRATION_ITERATOR_HPP

#include <memory>

#include "ilqr/integration/stepper/integration_stepper.hpp"
#include "ilqr/systems/continuous_time/continuous_system.hpp"

namespace ilqr {
namespace integration {
namespace iterator {

using ilqr::integration::stepper::IntegrationStepper;
using ilqr::systems::continuous_time::ContinuousSystem;

template <std::size_t N, std::size_t M>
class IntegrationIterator {
   public:
    using StepperPtr = std::shared_ptr<IntegrationStepper<N, M>>;
    using SystemPtr = std::shared_ptr<ContinuousSystem<N, M>>;
    using State = Eigen::Matrix<double, N, 1>;
    using Control = Eigen::Matrix<double, M, 1>;
    using Time = double;

    constexpr std::size_t GetNumState() const { return N; }

    constexpr std::size_t GetNumControl() const { return M; }

    virtual void Next(const StepperPtr& stepper, const SystemPtr& system,
                      const State& state, const Control& control,
                      const Time& time, const Time& delta_time,
                      State& result) const = 0;
};
}  // namespace iterator
}  // namespace integration
}  // namespace ilqr
#endif  // ILQR_INTEGRATION_ITERATOR_INTEGRATION_ITERATOR_HPP