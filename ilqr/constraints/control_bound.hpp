#ifndef ILQR_CONSTRAINTS_CONTROL_BOUND_HPP
#define ILQR_CONSTRAINTS_CONTROL_BOUND_HPP

#include <ilqr/constraints/inequality_constraint.hpp>

namespace ilqr {
namespace constraints {

template <std::size_t N, std::size_t M>
class ControlBound : public InequalityConstraint<N, M> {
   public:
    using State = typename InequalityConstraint<N, M>::State;
    using Control = typename InequalityConstraint<N, M>::Control;
    using Index = typename InequalityConstraint<N, M>::Index;

    ControlBound(const Control& lower_bound, const Control& upper_bound)
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
        for (std::size_t i = 0; i < M; ++i) {
            running_cost -=
                alpha_ *
                (std::log((upper_bound_(i) - control(i)) / diff_bound_(i)) +
                 std::log((control(i) - lower_bound_(i)) / diff_bound_(i)) +
                 std::log(4.0));
        }
        return running_cost;
    }

    double CalculateTerminalCost(const State& state,
                                 const Index& index) const override {
        return 0.0;
    }

    void CalculateRunningCostStateGradient(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, N, 1>& gradient) const override {
        gradient.setZero();
    }

    void CalculateRunningCostControlGradient(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, M, 1>& gradient) const override {
        gradient.setZero();
        for (std::size_t i = 0; i < M; ++i) {
            gradient(i) = alpha_ * (1.0 / (upper_bound_(i) - control(i)) -
                                    1.0 / (control(i) - lower_bound_(i)));
        }
    }

    void CalculateRunningCostStateHessian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, N, N>& hessian) const override {
        hessian.setZero();
    }

    void CalculateRunningCostControlHessian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, M, M>& hessian) const override {
        hessian.setZero();
        for (std::size_t i = 0; i < M; ++i) {
            hessian(i, i) =
                alpha_ * (1.0 / std::pow(upper_bound_(i) - control(i), 2) +
                          1.0 / std::pow(control(i) - lower_bound_(i), 2));
        }
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
    }

    void CalculateTerminalCostStateHessian(
        const State& state, const Index& index,
        Eigen::Matrix<double, N, N>& hessian) const override {
        hessian.setZero();
    }

   private:
    Control lower_bound_;
    Control upper_bound_;
    Control diff_bound_;
    double alpha_ = 0.01;
};

}  // namespace constraints
}  // namespace ilqr

#endif  // ILQR_CONSTRAINTS_CONTROL_BOUND_HPP