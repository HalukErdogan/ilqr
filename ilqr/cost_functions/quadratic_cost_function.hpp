#ifndef ILQR_COST_FUNCTIONS_QUADRATIC_COST_FUNCTION_HPP
#define ILQR_COST_FUNCTIONS_QUADRATIC_COST_FUNCTION_HPP

#include <array>

#include "ilqr/cost_functions/cost_function.hpp"

namespace ilqr {
namespace cost_functions {

template <std::size_t N, std::size_t M, std::size_t H>
class QuadraticCostFunction : public CostFunction<N, M> {
   public:
    using State = typename CostFunction<N, M>::State;
    using Control = typename CostFunction<N, M>::Control;
    using Index = typename CostFunction<N, M>::Index;

    QuadraticCostFunction(
        const std::array<State, H>& reference_states,
        const std::array<Control, H - 1>& reference_controls,
        const Eigen::Matrix<double, N, N>& terminal_state_weight,
        const Eigen::Matrix<double, N, N>& running_state_weight,
        const Eigen::Matrix<double, M, M>& running_control_weight)
        : reference_states_(reference_states),
          reference_controls_(reference_controls),
          terminal_state_weight_(terminal_state_weight),
          running_state_weight_(running_state_weight),
          running_control_weight_(running_control_weight) {}

    double CalculateRunningCost(const State& state, const Control& control,
                                const Index& index) const override {
        // Static variables to avoid memory allocation.
        static State state_error;
        static Control control_error;

        state_error = state - reference_states_[index];
        control_error = control - reference_controls_[index];
        return (0.5 * state_error.transpose() * running_state_weight_ *
                    state_error +
                0.5 * control_error.transpose() * running_control_weight_ *
                    control_error)
            .value();
    }

    double CalculateTerminalCost(const State& state,
                                 const Index& index) const override {
        // Static variable to avoid memory allocation.
        static State state_error;

        state_error = state - reference_states_[index];
        return (0.5 * state_error.transpose() * terminal_state_weight_ *
                state_error)
            .value();
    }

    void CalculateRunningCostStateGradient(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, N, 1>& gradient) const override {
        gradient = running_state_weight_ * (state - reference_states_[index]);
    }

    void CalculateRunningCostControlGradient(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, M, 1>& gradient) const override {
        gradient =
            running_control_weight_ * (control - reference_controls_[index]);
    }

    void CalculateRunningCostStateHessian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, N, N>& hessian) const override {
        hessian = running_state_weight_;
    }

    void CalculateRunningCostControlHessian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, M, M>& hessian) const override {
        hessian = running_control_weight_;
    }

    void CalculateRunningCostControlStateHessian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, M, N>& hessian) const override {
        hessian.setZero();
    }

    void CalculateTerminalCostStateGradient(
        const State& state, const Index& index,
        Eigen::Matrix<double, N, 1>& gradient) const override {
        gradient = terminal_state_weight_ * (state - reference_states_[index]);
    }

    void CalculateTerminalCostStateHessian(
        const State& state, const Index& index,
        Eigen::Matrix<double, N, N>& hessian) const override {
        hessian = terminal_state_weight_;
    }

   private:
    std::array<State, H> reference_states_;
    std::array<Control, H - 1> reference_controls_;
    Eigen::Matrix<double, N, N> terminal_state_weight_;
    Eigen::Matrix<double, N, N> running_state_weight_;
    Eigen::Matrix<double, M, M> running_control_weight_;
};
}  // namespace cost_functions
}  // namespace ilqr
#endif  // ILQR_COST_FUNCTIONS_QUADRATIC_COST_FUNCTION_HPP