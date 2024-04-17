#ifndef ILQR_COST_FUNCTIONS_COST_FUNCTION_WITH_FINITE_DIFF_HPP
#define ILQR_COST_FUNCTIONS_COST_FUNCTION_WITH_FINITE_DIFF_HPP

#include "ilqr/cost_functions/cost_function.hpp"

namespace ilqr {
namespace cost_functions {

template <std::size_t N, std::size_t M>
class CostFunctionWithFiniteDiff : public CostFunction<N, M> {
   public:
    using State = typename CostFunction<N, M>::State;
    using Control = typename CostFunction<N, M>::Control;
    using Index = typename CostFunction<N, M>::Index;

    virtual double CalculateRunningCost(const State& state,
                                        const Control& control,
                                        const Index& index) const = 0;

    virtual double CalculateTerminalCost(const State& state,
                                         const Index& index) const = 0;

    void CalculateRunningCostStateGradient(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, N, 1>& gradient) const override {
        // Static variables to avoid memory allocation.
        static double cost_plus;
        static double cost_nominal;
        static State state_plus;

        // Calculate the nominal cost.
        cost_nominal = CalculateRunningCost(state, control, index);

        // Calculate the state gradient using finite difference.
        state_plus = state;
        for (std::size_t i = 0; i < N; ++i) {
            state_plus(i) += epsilon_;
            cost_plus = CalculateRunningCost(state_plus, control, index);
            gradient(i) = (cost_plus - cost_nominal) / epsilon_;
            state_plus(i) = state(i);
        }
    }

    void CalculateRunningCostControlGradient(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, M, 1>& gradient) const override {
        // Static variables to avoid memory allocation.
        static double cost_plus;
        static double cost_nominal;
        static Control control_plus;

        // Calculate the nominal cost.
        cost_nominal = CalculateRunningCost(state, control, index);

        // Calculate the control gradient using finite difference.
        control_plus = control;
        for (std::size_t i = 0; i < M; ++i) {
            control_plus(i) += epsilon_;
            cost_plus = CalculateRunningCost(state, control_plus, index);
            gradient(i) = (cost_plus - cost_nominal) / epsilon_;
            control_plus(i) = control(i);
        }
    }

    void CalculateRunningCostStateHessian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, N, N>& hessian) const override {
        // Static variables to avoid memory allocation.
        static Eigen::Matrix<double, N, 1> gradient_plus;
        static Eigen::Matrix<double, N, 1> gradient_nominal;
        static State state_plus;

        // Calculate the nominal state gradient.
        CalculateRunningCostStateGradient(state, control, index, gradient_nominal);

        // Calculate the state hessian using finite difference.
        state_plus = state;
        for (std::size_t i = 0; i < N; ++i) {
            state_plus(i) += epsilon_;
            CalculateRunningCostStateGradient(state_plus, control, index,
                                          gradient_plus);
            hessian.col(i) = (gradient_plus - gradient_nominal) / epsilon_;
            state_plus(i) = state(i);
        }
    }

    void CalculateRunningCostControlHessian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, M, M>& hessian) const override {
        // Static variables to avoid memory allocation.
        static Eigen::Matrix<double, M, 1> gradient_plus;
        static Eigen::Matrix<double, M, 1> gradient_nominal;
        static Control control_plus;

        // Calculate the nominal control gradient.
        CalculateRunningCostControlGradient(state, control, index,
                                        gradient_nominal);

        // Calculate the control hessian using finite difference.
        control_plus = control;
        for (std::size_t i = 0; i < M; ++i) {
            control_plus(i) += epsilon_;
            CalculateRunningCostControlGradient(state, control_plus, index,
                                            gradient_plus);
            hessian.col(i) = (gradient_plus - gradient_nominal) / epsilon_;
            control_plus(i) = control(i);
        }
    }

    void CalculateRunningCostControlStateHessian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, M, N>& hessian) const override {
        // Static variables to avoid memory allocation.
        static Eigen::Matrix<double, M, 1> gradient_plus;
        static Eigen::Matrix<double, M, 1> gradient_nominal;
        static State state_plus;

        // Calculate the nominal state gradient.
        CalculateRunningCostControlGradient(state, control, index, gradient_nominal);

        // Calculate the control hessian using finite difference.
        state_plus = state;
        for (std::size_t i = 0; i < N; ++i) {
            state_plus(i) += epsilon_;
            CalculateRunningCostControlGradient(state_plus, control, index,
                                            gradient_plus);
            hessian.col(i) = (gradient_plus - gradient_nominal) / epsilon_;
            state_plus(i) = state(i);
        }
    }

    void CalculateTerminalCostStateGradient(
        const State& state, const Index& index,
        Eigen::Matrix<double, N, 1>& gradient) const override {
        // Static variables to avoid memory allocation.
        static double cost_plus;
        static double cost_nominal;
        static State state_plus;

        // Calculate the nominal cost.
        cost_nominal = CalculateTerminalCost(state, index);

        // Calculate the state gradient using finite difference.
        state_plus = state;
        for (std::size_t i = 0; i < N; ++i) {
            state_plus(i) += epsilon_;
            cost_plus = CalculateTerminalCost(state_plus, index);
            gradient(i) = (cost_plus - cost_nominal) / epsilon_;
            state_plus(i) = state(i);
        }
    }

    void CalculateTerminalCostStateHessian(
        const State& state, const Index& index,
        Eigen::Matrix<double, N, N>& hessian) const override {
        // Static variables to avoid memory allocation.
        static Eigen::Matrix<double, N, 1> gradient_plus;
        static Eigen::Matrix<double, N, 1> gradient_nominal;
        static State state_plus;

        // Calculate the nominal state gradient.
        CalculateTerminalCostStateGradient(state, index, gradient_nominal);

        // Calculate the state hessian using finite difference.
        state_plus = state;
        for (std::size_t i = 0; i < N; ++i) {
            state_plus(i) += epsilon_;
            CalculateTerminalCostStateGradient(state_plus, index, gradient_plus);
            hessian.col(i) = (gradient_plus - gradient_nominal) / epsilon_;
            state_plus(i) = state(i);
        }
    }

   private:
    double epsilon_ = 1e-6;
};

}  // namespace cost_functions
}  // namespace ilqr

#endif  // ILQR_COST_FUNCTIONS_COST_FUNCTION_WITH_FINITE_DIFF_HPP