#ifndef ILQR_SYSTEMS_DISCRETE_TIME_DISCRETIZER_HPP
#define ILQR_SYSTEMS_DISCRETE_TIME_DISCRETIZER_HPP

#include <memory>

#include "ilqr/integration/iterator/integration_iterator.hpp"
#include "ilqr/integration/stepper/integration_stepper.hpp"
#include "ilqr/systems/continuous_time/continuous_system.hpp"
#include "ilqr/systems/discrete_time/discrete_system_with_finite_diff.hpp"

namespace ilqr {
namespace systems {
namespace discrete_time {

using ilqr::integration::iterator::IntegrationIterator;
using ilqr::integration::stepper::IntegrationStepper;
using ilqr::systems::continuous_time::ContinuousSystem;
using ilqr::systems::discrete_time::DiscreteSystemWithFiniteDiff;

template <std::size_t N, std::size_t M>
class Discretizer : public DiscreteSystemWithFiniteDiff<N, M> {
   public:
    using SystemPtr               = std::shared_ptr<ContinuousSystem<N, M>>;
    using IntegrationIteratorPtr  = std::shared_ptr<IntegrationIterator<N, M>>;
    using IntegrationStepperPtr   = std::shared_ptr<IntegrationStepper<N, M>>;
    using State                   = typename DiscreteSystemWithFiniteDiff<N, M>::State;
    using Control                 = typename DiscreteSystemWithFiniteDiff<N, M>::Control;
    using Index                   = typename DiscreteSystemWithFiniteDiff<N, M>::Index;
    using Time                    = double;

    Discretizer(const SystemPtr& continuous_system,
                const IntegrationIteratorPtr& iterator,
                const IntegrationStepperPtr& stepper,
                const double init_time = 0.0, const double delta_time = 0.01)
        : continuous_system_(continuous_system),
          iterator_(iterator),
          stepper_(stepper),
          init_time_{init_time},
          delta_time_{delta_time} {}

    void PropogateDynamics(const State& state, const Control& control,
                           const Index& index, State& result) const override {
        Time time = init_time_ + index * delta_time_;
        iterator_->Next(stepper_, continuous_system_, state, control, time,
                        delta_time_, result);
    }

   private:
    SystemPtr continuous_system_;
    IntegrationIteratorPtr iterator_;
    IntegrationStepperPtr stepper_;
    double init_time_;
    double delta_time_;
};

}  // namespace discrete_time
}  // namespace systems
}  // namespace ilqr

#endif  // ILQR_SYSTEMS_DISCRETE_TIME_DISCRETIZER_HPP