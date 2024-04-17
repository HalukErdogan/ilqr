#ifndef ILQR_SYSTEMS_CONTINUOUS_TIME_CONTINUOUS_SYSTEM_HPP
#define ILQR_SYSTEMS_CONTINUOUS_TIME_CONTINUOUS_SYSTEM_HPP

#include <Eigen/Core>

namespace ilqr {
namespace systems {
namespace continuous_time {

template <std::size_t N, std::size_t M>
class ContinuousSystem {
   public:
    using State = Eigen::Matrix<double, N, 1>;
    using Control = Eigen::Matrix<double, M, 1>;
    using Time = double;

    constexpr std::size_t GetNumState() const { return N; }

    constexpr std::size_t GetNumControl() const { return M; }

    virtual void CalculateDynamics(const State& state, const Control& control,
                                   const Time& time, State& dynamics) const = 0;

    virtual void CalculateStateJacobian(
        const State& state, const Control& control, const Time& time,
        Eigen::Matrix<double, N, N>& jacobian) const {
        throw std::runtime_error("Not implemented.");
    }

    virtual void CalculateControlJacobian(
        const State& state, const Control& control, const Time& time,
        Eigen::Matrix<double, N, M>& jacobian) const {
        throw std::runtime_error("Not implemented.");
    }
};

}  // namespace continuous_time
}  // namespace systems
}  // namespace ilqr

#endif  // ILQR_SYSTEMS_CONTINUOUS_TIME_CONTINUOUS_SYSTEM_HPP