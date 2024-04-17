#include <gtest/gtest.h>

#include "ilqr/cost_functions/cost_function_with_finite_diff.hpp"

using ilqr::cost_functions::CostFunctionWithFiniteDiff;

class QuadraticCost : public CostFunctionWithFiniteDiff<2, 1> {
   public:
    QuadraticCost() {
        running_state_weight_   << 1.0, 0.0, 0.0, 1.0;
        running_control_weight_ << 1.0;
        terminal_state_weight_  << 1.0, 0.0, 0.0, 1.0;
    }

    double CalculateRunningCost(const State& state, const Control& control,
                              const Index& index) const override {
        return (0.5 * state.transpose() * running_state_weight_ * state +
                0.5 * control.transpose() * running_control_weight_ * control)
            .value();
    }

    double CalculateTerminalCost(const State& state,
                               const Index& index) const override {
        return (0.5 * state.transpose() * terminal_state_weight_ * state)
            .value();
    }

   private:
    Eigen::Matrix<double, 2, 2> running_state_weight_;
    Eigen::Matrix<double, 1, 1> running_control_weight_;
    Eigen::Matrix<double, 2, 2> terminal_state_weight_;
};

TEST(CostFunctionWithFiniteDiff, CalculateRunningCostStateGradient) {
    QuadraticCost cost;
    QuadraticCost::State state;
    state << 1.0, 1.0;
    QuadraticCost::Control control;
    control << 1.0;
    QuadraticCost::Index index = 0;
    Eigen::Matrix<double, 2, 1> gradient;
    cost.CalculateRunningCostStateGradient(state, control, index, gradient);

    Eigen::Matrix<double, 2, 1> gradient_expected;
    gradient_expected << 1.0, 1.0;

    ASSERT_TRUE(gradient.isApprox(gradient_expected, 1e-6));
}

TEST(CostFunctionWithFiniteDiff, CalculateRunningCostControlGradient) {
    QuadraticCost cost;
    QuadraticCost::State state;
    state << 1.0, 1.0;
    QuadraticCost::Control control;
    control << 1.0;
    QuadraticCost::Index index = 0;
    Eigen::Matrix<double, 1, 1> gradient;
    cost.CalculateRunningCostControlGradient(state, control, index, gradient);

    Eigen::Matrix<double, 1, 1> gradient_expected;
    gradient_expected << 1.0;

    ASSERT_TRUE(gradient.isApprox(gradient_expected, 1e-6));
}

TEST(CostFunctionWithFiniteDiff, CalculateRunningCostStateHessian) {
    QuadraticCost cost;
    QuadraticCost::State state;
    state << 1.0, 1.0;
    QuadraticCost::Control control;
    control << 1.0;
    QuadraticCost::Index index = 0;
    Eigen::Matrix<double, 2, 2> hessian;
    cost.CalculateRunningCostStateHessian(state, control, index, hessian);

    Eigen::Matrix<double, 2, 2> hessian_expected;
    hessian_expected << 1.0, 0.0, 0.0, 1.0;

    ASSERT_TRUE(hessian.isApprox(hessian_expected, 1e-3));
}

TEST(CostFunctionWithFiniteDiff, CalculateRunningCostControlHessian) {
    QuadraticCost cost;
    QuadraticCost::State state;
    state << 1.0, 1.0;
    QuadraticCost::Control control;
    control << 1.0;
    QuadraticCost::Index index = 0;
    Eigen::Matrix<double, 1, 1> hessian;
    cost.CalculateRunningCostControlHessian(state, control, index, hessian);

    Eigen::Matrix<double, 1, 1> hessian_expected;
    hessian_expected << 1.0;

    ASSERT_TRUE(hessian.isApprox(hessian_expected, 1e-3));
}

TEST(CostFunctionWithFiniteDiff, CalculateRunningCostControlStateHessian) {
    QuadraticCost cost;
    QuadraticCost::State state;
    state << 1.0, 1.0;
    QuadraticCost::Control control;
    control << 1.0;
    QuadraticCost::Index index = 0;
    Eigen::Matrix<double, 1, 2> hessian;
    cost.CalculateRunningCostControlStateHessian(state, control, index, hessian);

    Eigen::Matrix<double, 1, 2> hessian_expected;
    hessian_expected << 0.0, 0.0;

    ASSERT_TRUE(hessian.isApprox(hessian_expected, 1e-3));
}

TEST(CostFunctionWithFiniteDiff, CalculateTerminalCostStateGradient) {
    QuadraticCost cost;
    QuadraticCost::State state;
    state << 1.0, 1.0;
    QuadraticCost::Index index = 0;
    Eigen::Matrix<double, 2, 1> gradient;
    cost.CalculateTerminalCostStateGradient(state, index, gradient);

    Eigen::Matrix<double, 2, 1> gradient_expected;
    gradient_expected << 1.0, 1.0;

    ASSERT_TRUE(gradient.isApprox(gradient_expected, 1e-6));
}

TEST(CostFunctionWithFiniteDiff, CalculateTerminalCostStateHessian) {
    QuadraticCost cost;
    QuadraticCost::State state;
    state << 1.0, 1.0;
    QuadraticCost::Index index = 0;
    Eigen::Matrix<double, 2, 2> hessian;
    cost.CalculateTerminalCostStateHessian(state, index, hessian);

    Eigen::Matrix<double, 2, 2> hessian_expected;
    hessian_expected << 1.0, 0.0, 0.0, 1.0;

    ASSERT_TRUE(hessian.isApprox(hessian_expected, 1e-3));
}