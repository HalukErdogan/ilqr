#ifndef ILQR_SYSTEMS_CONTINUOUS_TIME_CONTINUOUS_SYSTEM_WITH_FINITE_DIFF_HPP
#define ILQR_SYSTEMS_CONTINUOUS_TIME_CONTINUOUS_SYSTEM_WITH_FINITE_DIFF_HPP

#include "ilqr/systems/continuous_time/continuous_system.hpp"

namespace ilqr {
namespace systems {
namespace continuous_time {

template <std::size_t N, std::size_t M>
class ContinousSystemWithFiniteDiff : public ContinousSystem<N, M> {
   public:
    using State = typename ContinousSystem<N, M>::State;
    using Control = typename ContinousSystem<N, M>::Control;
    using Time = typename ContinousSystem<N, M>::Time;

    virtual void CalculateDynamics(const State& state, const Control& control,
                                   const Time& time,
                                   State& result) const override {
        throw std::runtime_error("Not implemented.");
    }

    void CalculateStateJacobian(
        const State& state, const Control& control, const Time& time,
        Eigen::Matrix<double, N, N>& jacobian) const override {
        // Static variables to avoid memory allocation.
        static State dynamics_nominal;
        static State dynamics_plus;
        static State state_plus;

        // Calculate the nominal dynamics.
        CalculateDynamics(state, control, time, dynamics_nominal);

        // Calculate the state jacobian using finite difference.
        state_plus = state;
        for (std::size_t i = 0; i < N; ++i) {
            state_plus(i) += epsilon_;
            CalculateDynamics(state_plus, control, time, dynamics_plus);
            jacobian.col(i) = (dynamics_plus - dynamics_nominal) / epsilon_;
            state_plus(i) = state(i);
        }
    }

    void CalculateControlJacobian(
        const State& state, const Control& control, const Time& time,
        Eigen::Matrix<double, N, M>& jacobian) const override {
        // Static variables to avoid memory allocation.
        static State dynamics_nominal;
        static State dynamics_plus;
        static Control control_plus;

        // Calculate the nominal dynamics.
        CalculateDynamics(state, control, time, dynamics_nominal);

        // Calculate the control jacobian using finite difference.
        control_plus = control;
        for (std::size_t i = 0; i < M; ++i) {
            control_plus(i) += epsilon_;
            CalculateDynamics(state, control_plus, time, dynamics_plus);
            jacobian.col(i) = (dynamics_plus - dynamics_nominal) / epsilon_;
            control_plus(i) = control(i);
        }
    }

   private:
    double epsilon_ = 1e-6;
};

}  // namespace continuous_time
}  // namespace systems
}  // namespace ilqr

#endif  // ILQR_SYSTEMS_CONTINUOUS_TIME_CONTINUOUS_SYSTEM_WITH_FINITE_DIFF_HPP