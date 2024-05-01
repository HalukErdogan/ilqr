#ifndef ILQR_COST_FUNCTIONS_COST_FUNCTION_WITH_CONSTRAINTS_HPP
#define ILQR_COST_FUNCTIONS_COST_FUNCTION_WITH_CONSTRAINTS_HPP

#include <ilqr/constraints/inequality_constraint.hpp>
#include <ilqr/cost_functions/cost_function.hpp>

#include <memory>
#include <vector>

namespace ilqr {
namespace cost_functions {

template <std::size_t N, std::size_t M>
class CostFunctionWithConstraints : public CostFunction<N, M> {
   public:
    using State = typename CostFunction<N, M>::State;
    using Control = typename CostFunction<N, M>::Control;
    using Index = typename CostFunction<N, M>::Index;

    CostFunctionWithConstraints(
        const std::shared_ptr<CostFunction<N, M>>& cost_function,
        const std::vector<
            std::shared_ptr<constraints::InequalityConstraint<N, M>>>&
            constraints)
        : cost_function_(cost_function), constraints_(constraints) {}

    double CalculateRunningCost(const State& state, const Control& control,
                                const Index& index) const override {
        // Calculate the running cost.
        double running_cost =
            cost_function_->CalculateRunningCost(state, control, index);

        // Add the running cost of the constraints.
        for (const auto& constraint : constraints_) {
            running_cost +=
                constraint->CalculateRunningCost(state, control, index);
        }

        return running_cost;
    }

    double CalculateTerminalCost(const State& state,
                                 const Index& index) const override {
        // Calculate the terminal cost.
        double terminal_cost =
            cost_function_->CalculateTerminalCost(state, index);
        
        // Add the terminal cost of the constraints.
        for (const auto& constraint : constraints_) {
            terminal_cost += constraint->CalculateTerminalCost(state, index);
        }

        return terminal_cost;
    }

    void CalculateRunningCostStateGradient(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, N, 1>& gradient) const override {
        // Static variables to avoid memory allocation.
        static Eigen::Matrix<double, N, 1> gradient_constraint;

        // Calculate the state gradient of the running cost.
        cost_function_->CalculateRunningCostStateGradient(state, control, index,
                                                          gradient);

        // Add the state gradient of the constraints.
        for (const auto& constraint : constraints_) {
            constraint->CalculateRunningCostStateGradient(state, control, index,
                                                          gradient_constraint);
            gradient += gradient_constraint;
        }
    }

    void CalculateRunningCostControlGradient(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, M, 1>& gradient) const override {
        // Static variables to avoid memory allocation.
        static Eigen::Matrix<double, M, 1> gradient_constraint;

        // Calculate the control gradient of the running cost.
        cost_function_->CalculateRunningCostControlGradient(state, control, index,
                                                            gradient);

        // Add the control gradient of the constraints.
        for (const auto& constraint : constraints_) {
            constraint->CalculateRunningCostControlGradient(state, control, index,
                                                            gradient_constraint);
            gradient += gradient_constraint;
        }
    }

    void CalculateRunningCostStateHessian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, N, N>& hessian) const override {
        // Static variables to avoid memory allocation.
        static Eigen::Matrix<double, N, N> hessian_constraint;

        // Calculate the state hessian of the running cost.
        cost_function_->CalculateRunningCostStateHessian(state, control, index,
                                                        hessian);

        // Add the state hessian of the constraints.
        for (const auto& constraint : constraints_) {
            constraint->CalculateRunningCostStateHessian(state, control, index,
                                                        hessian_constraint);
            hessian += hessian_constraint;
        }
    }

    void CalculateRunningCostControlHessian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, M, M>& hessian) const override {
        // Static variables to avoid memory allocation.
        static Eigen::Matrix<double, M, M> hessian_constraint;

        // Calculate the control hessian of the running cost.
        cost_function_->CalculateRunningCostControlHessian(state, control, index,
                                                          hessian);

        // Add the control hessian of the constraints.
        for (const auto& constraint : constraints_) {
            constraint->CalculateRunningCostControlHessian(state, control, index,
                                                          hessian_constraint);
            hessian += hessian_constraint;
        }
    }

    void CalculateRunningCostControlStateHessian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, M, N>& hessian) const override {
        // Static variables to avoid memory allocation.
        static Eigen::Matrix<double, M, N> hessian_constraint;

        // Calculate the state-control hessian of the running cost.
        cost_function_->CalculateRunningCostControlStateHessian(state, control, index,
                                                              hessian);

        // Add the state-control hessian of the constraints.
        for (const auto& constraint : constraints_) {
            constraint->CalculateRunningCostControlStateHessian(state, control, index,
                                                              hessian_constraint);
            hessian += hessian_constraint;
        }
    }

    void CalculateTerminalCostStateGradient(
        const State& state, const Index& index,
        Eigen::Matrix<double, N, 1>& gradient) const override {
        // Static variables to avoid memory allocation.
        static Eigen::Matrix<double, N, 1> gradient_constraint;

        // Calculate the state gradient of the terminal cost.
        cost_function_->CalculateTerminalCostStateGradient(state, index,
                                                           gradient);

        // Add the state gradient of the constraints.
        for (const auto& constraint : constraints_) {
            constraint->CalculateTerminalCostStateGradient(state, index,
                                                           gradient_constraint);
            gradient += gradient_constraint;
        }
    }

    void CalculateTerminalCostStateHessian(
        const State& state, const Index& index,
        Eigen::Matrix<double, N, N>& hessian) const override {
        // Static variables to avoid memory allocation.
        static Eigen::Matrix<double, N, N> hessian_constraint;

        // Calculate the state hessian of the terminal cost.
        cost_function_->CalculateTerminalCostStateHessian(state, index,
                                                        hessian);

        // Add the state hessian of the constraints.
        for (const auto& constraint : constraints_) {
            constraint->CalculateTerminalCostStateHessian(state, index,
                                                        hessian_constraint);
            hessian += hessian_constraint;
        }
    }

   private:
    std::shared_ptr<CostFunction<N, M>> cost_function_;
    std::vector<std::shared_ptr<constraints::InequalityConstraint<N, M>>>
        constraints_;
};

}  // namespace cost_functions
}  // namespace ilqr

#endif  // ILQR_COST_FUNCTIONS_COST_FUNCTION_WITH_CONSTRAINTS_HPP