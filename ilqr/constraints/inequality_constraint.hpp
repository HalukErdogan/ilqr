#ifndef ILQR_CONSTRAINTS_INEQUALITY_CONSTRAINT_HPP
#define ILQR_CONSTRAINTS_INEQUALITY_CONSTRAINT_HPP

#include "ilqr\cost_functions\cost_function_with_finite_diff.hpp"

#include <Eigen/Core>

namespace ilqr {
namespace constraints {

using ilqr::cost_functions::CostFunctionWithFiniteDiff;

template <std::size_t N, std::size_t M>
class InequalityConstraint : public CostFunctionWithFiniteDiff<N, M> {
   public:
    using State = Eigen::Matrix<double, N, 1>;
    using Control = Eigen::Matrix<double, M, 1>;
    using Index = std::size_t;

    constexpr std::size_t GetNumState() const { return N; }

    constexpr std::size_t GetNumControl() const { return M; }

    virtual double CalculateRunningInequalityConstraint(
        const State& state, const Control& control,
        const Index& index) const = 0;

    virtual double CalculateTerminalInequalityConstraint(
        const State& state, const Index& index) const = 0;

    double CalculateRunningCost(const State& state, const Control& control, const Index& index) const override {
        // Logarithmic barrier function.
        return -std::log(-CalculateRunningInequalityConstraint(state, control, index));
    }

    double CalculateTerminalCost(const State& state, const Index& index) const override {
        // Logarithmic barrier function.
        return -std::log(-CalculateTerminalInequalityConstraint(state, index));
    }
};

}  // namespace constraints
}  // namespace ilqr

#endif  // ILQR_CONSTRAINTS_INEQUALITY_CONSTRAINT_HPP