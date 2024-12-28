#ifndef ILQR_CONSTRAINTS_STATE_BOUND_HPP
#define ILQR_CONSTRAINTS_STATE_BOUND_HPP

#include <ilqr/constraints/inequality_constraint.hpp>

namespace ilqr {
namespace constraints {

template <std::size_t N, std::size_t M>
class StateBound : public InequalityConstraint<N, M> {
   public:
    using State = typename InequalityConstraint<N, M>::State;
    using Control = typename InequalityConstraint<N, M>::Control;
    using Index = typename InequalityConstraint<N, M>::Index;

    StateBound(const State& lower_bound, const State& upper_bound)
        : lower_bound_(lower_bound),
          upper_bound_(upper_bound),
          diff_bound_(upper_bound - lower_bound) {}

    double CalculateRunningInequalityConstraint(
        const State& state, const Control& control,
        const Index& index) const override {
        return 0.0;
    }

    double CalculateTerminalInequalityConstraint(
        const State& state, const Index& index) const override {
        return 0.0;
    }

    double CalculateRunningCost(const State& state, const Control& control,
                                const Index& index) const override {
        // Logarithmic barrier function.
        double running_cost = 0.0;
        for (std::size_t i = 0; i < N; ++i) {
            running_cost -=
                alpha_ *
                (std::log((upper_bound_(i) - state(i)) / diff_bound_(i)) +
                 std::log((state(i) - lower_bound_(i)) / diff_bound_(i)) +
                 std::log(4.0));
        }
        return running_cost;
    }

    double CalculateTerminalCost(const State& state,
                                 const Index& index) const override {
        double terminal_cost = 0.0;
        for (std::size_t i = 0; i < N; ++i) {
            terminal_cost -=
                alpha_ *
                (std::log((upper_bound_(i) - state(i)) / diff_bound_(i)) +
                 std::log((state(i) - lower_bound_(i)) / diff_bound_(i)) +
                 std::log(4.0));
        }
        return terminal_cost;
    }

    void CalculateRunningCostStateGradient(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, N, 1>& gradient) const override {
        gradient.setZero();
        for (std::size_t i = 0; i < N; ++i) {
            gradient(i) = alpha_ * (1.0 / (upper_bound_(i) - state(i)) -
                                    1.0 / (state(i) - lower_bound_(i)));
        }
    }

    void CalculateRunningCostControlGradient(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, M, 1>& gradient) const override {
        gradient.setZero();
    }

    void CalculateRunningCostStateHessian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, N, N>& hessian) const override {
        hessian.setZero();
        for (std::size_t i = 0; i < N; ++i) {
            hessian(i, i) =
                alpha_ * (1.0 / std::pow(upper_bound_(i) - state(i), 2) +
                          1.0 / std::pow(state(i) - lower_bound_(i), 2));
        }
    }

    void CalculateRunningCostControlHessian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, M, M>& hessian) const override {
        hessian.setZero();
    }

    void CalculateRunningCostControlStateHessian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, M, N>& hessian) const override {
        hessian.setZero();
    }

    void CalculateTerminalCostStateGradient(
        const State& state, const Index& index,
        Eigen::Matrix<double, N, 1>& gradient) const override {
        gradient.setZero();
        for (std::size_t i = 0; i < N; ++i) {
            gradient(i) = alpha_ * (1.0 / (upper_bound_(i) - state(i)) -
                                    1.0 / (state(i) - lower_bound_(i)));
        }
    }

    void CalculateTerminalCostStateHessian(
        const State& state, const Index& index,
        Eigen::Matrix<double, N, N>& hessian) const override {
        hessian.setZero();
        for (std::size_t i = 0; i < N; ++i) {
            hessian(i, i) =
                alpha_ * (1.0 / std::pow(upper_bound_(i) - state(i), 2) +
                          1.0 / std::pow(state(i) - lower_bound_(i), 2));
        }
    }

   private:
    State lower_bound_;
    State upper_bound_;
    State diff_bound_;
    double alpha_ = 0.01;
};

}  // namespace constraints
}  // namespace ilqr

#endif  // ILQR_CONSTRAINTS_STATE_BOUND_HPP