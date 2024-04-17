#ifndef ILQR_SYSTEMS_DISCRETE_TIME_DISCRETE_SYSTEM_WITH_FINITE_DIFF_HPP
#define ILQR_SYSTEMS_DISCRETE_TIME_DISCRETE_SYSTEM_WITH_FINITE_DIFF_HPP

#include "ilqr/systems/discrete_time/discrete_system.hpp"

namespace ilqr {
namespace systems {
namespace discrete_time {

template <std::size_t N, std::size_t M>
class DiscreteSystemWithFiniteDiff : public DiscreteSystem<N, M> {
   public:
    using State = typename DiscreteSystem<N, M>::State;
    using Control = typename DiscreteSystem<N, M>::Control;
    using Index = typename DiscreteSystem<N, M>::Index;

    virtual void PropogateDynamics(const State& state, const Control& control,
                                   const Index& index,
                                   State& next_state) const override {
        throw std::runtime_error("Not implemented.");
    }

    void CalculateStateJacobian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, N, N>& jacobian) const override {
        // Static variables to avoid memory allocation.
        static State next_state_nominal;
        static State next_state_plus;
        static State state_plus;

        // Calculate the nominal next state.
        PropogateDynamics(state, control, index, next_state_nominal);

        // Calculate the state jacobian using finite difference.
        state_plus = state;
        for (std::size_t i = 0; i < N; ++i) {
            state_plus(i) += epsilon_;
            PropogateDynamics(state_plus, control, index, next_state_plus);
            jacobian.col(i) = (next_state_plus - next_state_nominal) / epsilon_;
            state_plus(i) = state(i);
        }
    }

    void CalculateControlJacobian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, N, M>& jacobian) const override {
        // Static variables to avoid memory allocation.
        static State next_state_nominal;
        static State next_state_plus;
        static Control control_plus;

        // Calculate the nominal next state.
        PropogateDynamics(state, control, index, next_state_nominal);

        // Calculate the control jacobian using finite difference.
        control_plus = control;
        for (std::size_t i = 0; i < M; ++i) {
            control_plus(i) += epsilon_;
            PropogateDynamics(state, control_plus, index, next_state_plus);
            jacobian.col(i) = (next_state_plus - next_state_nominal) / epsilon_;
            control_plus(i) = control(i);
        }
    }

   private:
    double epsilon_ = 1e-6;
};

}  // namespace discrete_time
}  // namespace systems
}  // namespace ilqr

#endif  // ILQR_SYSTEMS_DISCRETE_TIME_DISCRETE_SYSTEM_WITH_FINITE_DIFF_HPP