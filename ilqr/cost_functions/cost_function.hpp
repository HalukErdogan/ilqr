#ifndef ILQR_COST_FUNCTIONS_COST_FUNCTION_HPP
#define ILQR_COST_FUNCTIONS_COST_FUNCTION_HPP

#include <Eigen/Core>

namespace ilqr {
namespace cost_functions {

template <std::size_t N, std::size_t M>
class CostFunction {
   public:
    using State = Eigen::Matrix<double, N, 1>;
    using Control = Eigen::Matrix<double, M, 1>;
    using Index = std::size_t;

    constexpr std::size_t GetNumState() const { return N; }

    constexpr std::size_t GetNumControl() const { return M; }

    virtual double CalculateRunningCost(const State& state,
                                        const Control& control,
                                        const Index& index) const = 0;

    virtual double CalculateTerminalCost(const State& state,
                                         const Index& index) const = 0;

    virtual void CalculateRunningCostStateGradient(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, N, 1>& gradient) const = 0;

    virtual void CalculateRunningCostControlGradient(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, M, 1>& gradient) const = 0;

    virtual void CalculateRunningCostStateHessian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, N, N>& hessian) const = 0;

    virtual void CalculateRunningCostControlHessian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, M, M>& hessian) const = 0;

    virtual void CalculateRunningCostControlStateHessian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, M, N>& hessian) const = 0;

    virtual void CalculateTerminalCostStateGradient(
        const State& state, const Index& index,
        Eigen::Matrix<double, N, 1>& gradient) const = 0;

    virtual void CalculateTerminalCostStateHessian(
        const State& state, const Index& index,
        Eigen::Matrix<double, N, N>& hessian) const = 0;
};

}  // namespace cost_functions
}  // namespace ilqr

#endif  // ILQR_COST_FUNCTIONS_COST_FUNCTION_HPP