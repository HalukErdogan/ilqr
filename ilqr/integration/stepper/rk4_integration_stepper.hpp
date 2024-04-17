#ifndef ILQR_INTEGRATION_STEPPER_RK4_INTEGRATION_STEPPER_HPP
#define ILQR_INTEGRATION_STEPPER_RK4_INTEGRATION_STEPPER_HPP

#include "ilqr/integration/stepper/integration_stepper.hpp"

namespace ilqr {
namespace integration {
namespace stepper {

template<std::size_t N, std::size_t M>
class RK4IntegrationStepper : public IntegrationStepper<N, M> {
   public:
    void Step(const SystemPtr& system, const State& state, const Control& control,
              const Time& time, const Time& delta_time,
              State& result) const override {
        // Static variables to avoid memory allocation.
        static State k1, k2, k3, k4;

        // Calculate the Runge-Kutta 4th order integration.
        system->CalculateDynamics(state, control, time, k1);
        system->CalculateDynamics(state + delta_time * k1 / 2.0, control, time + delta_time / 2.0, k2);
        system->CalculateDynamics(state + delta_time * k2 / 2.0, control, time + delta_time / 2.0, k3);
        system->CalculateDynamics(state + delta_time * k3, control, time + delta_time, k4);
        result = state + delta_time * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
    }
};

}  // namespace stepper
}  // namespace integration
}  // namespace ilqr

#endif  // ILQR_INTEGRATION_STEPPER_RK4_INTEGRATION_STEPPER_HPP