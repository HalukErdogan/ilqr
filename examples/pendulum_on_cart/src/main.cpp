#define _USE_MATH_DEFINES

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>

#include "ilqr/controller/ilqr.hpp"
#include "ilqr/cost_functions/quadratic_cost_function.hpp"
#include "ilqr/cost_functions/cost_function_with_constraints.hpp"
#include "ilqr/integration/iterator/constant_integration_iterator.hpp"
#include "ilqr/integration/stepper/euler_integration_stepper.hpp"
#include "ilqr/integration/stepper/rk2_integration_stepper.hpp"
#include "ilqr/integration/stepper/rk3_integration_stepper.hpp"
#include "ilqr/integration/stepper/rk4_integration_stepper.hpp"
#include "ilqr/systems/discrete_time/discretizer.hpp"
#include "ilqr/constraints/inequality_constraint.hpp"
#include "ilqr/constraints/state_bound.hpp"
#include "ilqr/constraints/control_bound.hpp"
#include "pendulum_on_cart.hpp"

using ilqr::controller::ILQR;
using ilqr::controller::Options;
using ilqr::controller::Problem;
using ilqr::controller::Solution;
using ilqr::cost_functions::QuadraticCostFunction;
using ilqr::cost_functions::CostFunctionWithConstraints;
using ilqr::examples::PendulumOnCart;
using ilqr::integration::iterator::ConstantIntegrationIterator;
using ilqr::integration::stepper::EulerIntegrationStepper;
using ilqr::integration::stepper::RK2IntegrationStepper;
using ilqr::integration::stepper::RK3IntegrationStepper;
using ilqr::integration::stepper::RK4IntegrationStepper;
using ilqr::systems::continuous_time::ContinuousSystem;
using ilqr::systems::discrete_time::Discretizer;
using ilqr::constraints::InequalityConstraint;
using ilqr::constraints::StateBound;
using ilqr::constraints::ControlBound;

// Define the constants
constexpr double t0 = 0.0;      // initial time
constexpr double tf = 10.0;     // final time
constexpr double dt = 0.1;      // time step
constexpr std::size_t N = 4;    // number of states
constexpr std::size_t M = 1;    // number of control inputs
constexpr std::size_t H = (tf - t0) / dt + 1;   // horizon

// Define the variables
Eigen::Vector<double, N> x0;                     // initial state
Eigen::Vector<double, N> xt;                     // target state
Eigen::Matrix<double, M, M> R;                   // running control gains
Eigen::Matrix<double, N, N> Q;                   // running state gains
Eigen::Matrix<double, N, N> Qt;                  // terminal state gain
std::array<Eigen::Vector<double, N>, H> Xr;      // reference states
std::array<Eigen::Vector<double, M>, H - 1> Ur;  // reference controls

// Define the constraints
Eigen::Vector<double, N> x_lower;
Eigen::Vector<double, N> x_upper;
Eigen::Vector<double, M> u_lower;
Eigen::Vector<double, M> u_upper;

int main(int argc, char** argv) {
    // Init state
    x0.setZero();
    x0 << 0.0, 0.0, 0.0, 0.0;

    // Target state
    xt.setZero();
    xt << 0.0, M_PI, 0.0, 0.0;

    // Running control gains
    R.setZero();
    R.diagonal() << 0.02;

    // Running state gains
    Q.setZero();
    Q.diagonal() << 1.0, 1.0, 1.0, 1.0;

    // Terminal state gain
    Qt.setZero();
    Qt.diagonal() << 200.0, 200.0, 200.0, 200.0;

    // Scale the running cost gains with dt so the
    // cost calculation wont change according to
    // discritization
    Q *= dt;
    R *= dt;

    // Reference states
    Xr.fill(xt);

    // Reference controls
    Ur.fill(Eigen::Vector<double, M>::Zero());

    // State bounds
    x_lower << -10.0, -M_PI, -10.0, -10.0;
    x_upper << 10.0, M_PI, 10.0, 10.0;
    
    // Control bounds
    u_lower << -2.0;
    u_upper << 2.0;

    // Setup options
    Options options;
    options.max_iter = 100;
    options.tolerance = 1e-3;
    options.regularization = 1e-6;
    options.warm_start = false;

    // Setup the system
    auto continuous_model = std::make_shared<PendulumOnCart>();
    auto integration_stepper =
        std::make_shared<EulerIntegrationStepper<N, M>>();
    auto integration_iterator =
        std::make_shared<ConstantIntegrationIterator<N, M>>();
    auto discrete_system = std::make_shared<Discretizer<N, M>>(
        continuous_model, integration_iterator, integration_stepper, t0, dt);

    // Setup the cost function
    auto cost_function =
        std::make_shared<QuadraticCostFunction<N, M, H>>(Xr, Ur, Qt, Q, R);

    // Setup the constraints
    // auto state_bound = std::make_shared<StateBound<N, M>>(
    //     x_lower, x_upper);
    auto control_bound = std::make_shared<ControlBound<N, M>>(
        u_lower, u_upper);
    std::vector<std::shared_ptr<InequalityConstraint<N, M>>> constraints;
    // constraints.push_back(state_bound);
    constraints.push_back(control_bound);

    // Setup the cost function with constraints
    auto cost_function_with_constraints = std::make_shared<CostFunctionWithConstraints<N, M>>(
        cost_function, constraints);
    
    // Create problem
    Problem<N, M> problem;


    // Set initial state
    problem.init_state = x0;

    // Set the system
    problem.system = discrete_system;

    // Set the cost function
    problem.cost_function = cost_function_with_constraints;

    // Setup solution
    Solution<N, M, H> solution;

    // Create controller
    ILQR<N, M, H> mpc;

    // Solve the problem
    auto start = std::chrono::steady_clock::now();
    mpc.Optimize(problem, options, solution);
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Elapsed time: " << elapsed_seconds.count() << "s\n";

    // Create a file output stream object
    std::ofstream file;
    file.open("output.csv");
    file << "t,q1,q2,dq1,dq2,u" << std::endl;

    // Store the results
    for (std::size_t i = 0; i + 1 < solution.states.size(); ++i) {
        // Write the current state to the file
        const auto& x = solution.states.at(i);
        const auto& u = solution.controls.at(i);
        file << t0 + i * dt << "," << x(0) << "," << x(1) << "," << x(2) << ","
             << x(3) << "," << u(0) << std::endl;
    }
    file.close();

    return 0;
}
