#ifndef ILQR_INTEGRATION_STEPPER_RK2_INTEGRATION_STEPPER_HPP
#define ILQR_INTEGRATION_STEPPER_RK2_INTEGRATION_STEPPER_HPP

#include "ilqr/integration/stepper/integration_stepper.hpp"

namespace ilqr {
namespace integration {
namespace stepper {

template <std::size_t N, std::size_t M>
class RK2IntegrationStepper : public IntegrationStepper<N, M> {
   public:
    using SystemPtr = typename IntegrationStepper<N, M>::SystemPtr;
    using State = typename IntegrationStepper<N, M>::State;
    using Control = typename IntegrationStepper<N, M>::Control;
    using Time = typename IntegrationStepper<N, M>::Time;

    void Step(const SystemPtr& system, const State& state,
              const Control& control, const Time& time, const Time& delta_time,
              State& result) const override {
        // Static variables to avoid memory allocation.
        static State k1, k2;

        // Calculate the Runge-Kutta 2nd order integration.
        system->CalculateDynamics(state, control, time, k1);
        system->CalculateDynamics(state + delta_time * k1 / 2.0, control,
                                  time + delta_time / 2.0, k2);
        result = state + delta_time * k2;
    }
};

}  // namespace stepper
}  // namespace integration
}  // namespace ilqr

#endif  // ILQR_INTEGRATION_STEPPER_RK2_INTEGRATION_STEPPER_HPP