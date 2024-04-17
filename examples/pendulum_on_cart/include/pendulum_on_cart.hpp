#ifndef ILQR_EXAMPLES_PENDULUM_ON_CART_INCLUDE_PENDULUM_ON_CART_HPP
#define ILQR_EXAMPLES_PENDULUM_ON_CART_INCLUDE_PENDULUM_ON_CART_HPP

#include <Eigen/Dense>

#include "ilqr/systems/continuous_time/continuous_system.hpp"

using ilqr::systems::continuous_time::ContinuousSystem;

namespace ilqr {
namespace examples {

class PendulumOnCart : public ContinuousSystem<4, 1> {
   public:
    void CalculateDynamics(const State& state, const Control& control,
                           const Time& time, State& result) const override {
        // System params
        constexpr double M1 = 1.0;
        constexpr double M2 = 0.5;
        constexpr double L = 1.0;
        constexpr double G = 9.81;

        // Intermediate vars to reduce computation
        const double sin_q2 = std::sin(state[1]);
        const double cos_q2 = std::cos(state[1]);
        const double A = L * M2 * sin_q2 * std::pow(state[3], 2);
        const double B = G * sin_q2;
        const double C = M1 + M2 * sin_q2 * sin_q2;

        // dxdt
        result[0] = state[2];
        result[1] = state[3];
        result[2] = (control[0] + A + M2 * B * cos_q2) / C;
        result[3] = -((control[0] + A) * cos_q2 + (M1 + M2) * B) / (C * L);
    }
};

}  // namespace examples
}  // namespace ilqr

#endif  // ILQR_EXAMPLES_PENDULUM_ON_CART_INCLUDE_PENDULUM_ON_CART_HPP