#ifndef ILQR_SYSTEMS_DISCRETE_TIME_DISCRETE_SYSTEM_HPP
#define ILQR_SYSTEMS_DISCRETE_TIME_DISCRETE_SYSTEM_HPP

#include <Eigen/Core>

namespace ilqr {
namespace systems {
namespace discrete_time {

template <std::size_t N, std::size_t M>
class DiscreteSystem {
   public:
    using State = Eigen::Matrix<double, N, 1>;
    using Control = Eigen::Matrix<double, M, 1>;
    using Index = std::size_t;

    constexpr std::size_t GetNumState() const { return N; }

    constexpr std::size_t GetNumControl() const { return M; }

    virtual void PropogateDynamics(const State& state, const Control& control,
                                   const Index& index,
                                   State& next_state) const = 0;

    virtual void CalculateStateJacobian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, N, N>& jacobian) const {
        throw std::runtime_error("Not implemented.");
    }

    virtual void CalculateControlJacobian(
        const State& state, const Control& control, const Index& index,
        Eigen::Matrix<double, N, M>& jacobian) const {
        throw std::runtime_error("Not implemented.");
    }
};

}  // namespace discrete_time
}  // namespace systems
}  // namespace ilqr

#endif  // ILQR_SYSTEMS_DISCRETE_TIME_DISCRETE_SYSTEM_HPP