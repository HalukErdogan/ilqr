#include <gtest/gtest.h>

#include "ilqr/systems/discrete_time/discrete_system_with_finite_diff.hpp"

using ilqr::systems::discrete_time::DiscreteSystemWithFiniteDiff;

class OneDimCart : public DiscreteSystemWithFiniteDiff<2, 1> {
   public:
    void PropogateDynamics(const State& state, const Control& control,
                           const Index& index, State& result) const override {
        const double& x = state[0];
        const double& v = state[1];
        const double& a = control[0];
        result[0] = x + v * dt_ + 0.5 * a * dt_ * dt_;
        result[1] = v + a * dt_;
    }

   private:
    double dt_ = 0.01;
};

TEST(DiscreteSystemWithFiniteDiff, CalculateStateJacobian) {
    OneDimCart cart;
    OneDimCart::State state;
    state << 0.0, 0.0;
    OneDimCart::Control control;
    control << 1.0;
    Eigen::Matrix<double, 2, 2> fx;
    cart.CalculateStateJacobian(state, control, 0, fx);

    Eigen::Matrix<double, 2, 2> fx_expected;
    fx_expected << 1.0, 0.01, 0.0, 1.0;

    ASSERT_TRUE(fx.isApprox(fx_expected, 1e-6));
}

TEST(DiscreteSystemWithFiniteDiff, CalculateControlJacobian) {
    OneDimCart cart;
    OneDimCart::State state;
    state << 0.0, 0.0;
    OneDimCart::Control control;
    control << 1.0;
    Eigen::Matrix<double, 2, 1> fu;
    cart.CalculateControlJacobian(state, control, 0, fu);

    Eigen::Matrix<double, 2, 1> fu_expected;
    fu_expected << 0.00005, 0.01;

    ASSERT_TRUE(fu.isApprox(fu_expected, 1e-6));
}