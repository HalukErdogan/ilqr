#ifndef ILQR_INTEGRATION_STEPPER_EULER_INTEGRATION_STEPPER_HPP
#define ILQR_INTEGRATION_STEPPER_EULER_INTEGRATION_STEPPER_HPP

#include "ilqr/integration/stepper/integration_stepper.hpp"

namespace ilqr {
namespace integration {
namespace stepper {

template <std::size_t N, std::size_t M>
class EulerIntegrationStepper : public IntegrationStepper<N, M> {
   public:
    void Step(const SystemPtr& system, const State& state, const Control& control,
              const Time& time, const Time& delta_time,
              State& result) const override {
        // Static variables to avoid memory allocation.
        static State k1;

        // Calculate the Euler integration.
        system->CalculateDynamics(state, control, time, k1);
        result = state + delta_time * k1;
    }
};

}  // namespace stepper
}  // namespace integration
}  // namespace ilqr

#endif  // ILQR_INTEGRATION_STEPPER_EULER_INTEGRATION_STEPPER_HPP