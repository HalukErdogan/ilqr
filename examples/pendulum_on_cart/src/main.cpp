#define _USE_MATH_DEFINES

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>

#include "ilqr/controller/ilqr.hpp"
#include "ilqr/cost_functions/quadratic_cost_function.hpp"
#include "ilqr/integration/iterator/constant_integration_iterator.hpp"
#include "ilqr/integration/stepper/euler_integration_stepper.hpp"
#include "ilqr/systems/discrete_time/discretizer.hpp"
#include "pendulum_on_cart.hpp"

using ilqr::controller::ILQR;
using ilqr::controller::Options;
using ilqr::controller::Problem;
using ilqr::controller::Solution;
using ilqr::cost_functions::QuadraticCostFunction;
using ilqr::examples::PendulumOnCart;
using ilqr::integration::iterator::ConstantIntegrationIterator;
using ilqr::integration::stepper::EulerIntegrationStepper;
using ilqr::systems::continuous_time::ContinuousSystem;
using ilqr::systems::discrete_time::Discretizer;

// Define the constants
constexpr std::size_t N = 4;    // number of states
constexpr std::size_t M = 1;    // number of control inputs
constexpr std::size_t H = 100;  // horizon
constexpr double t0 = 0.0;      // initial time
constexpr double dt = 0.1;      // time step

// Define the variables
Eigen::Vector<double, N> x0;                     // initial state
Eigen::Vector<double, N> xt;                     // target state
Eigen::Matrix<double, M, M> R;                   // running control gains
Eigen::Matrix<double, N, N> Q;                   // running state gains
Eigen::Matrix<double, N, N> Qt;                  // terminal state gain
std::array<Eigen::Vector<double, N>, H> Xr;      // reference states
std::array<Eigen::Vector<double, M>, H - 1> Ur;  // reference controls

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

    // Create problem
    Problem<N, M> problem;

    // Setup the cost function
    auto cost_function =
        std::make_shared<QuadraticCostFunction<N, M, H>>(Xr, Ur, Qt, Q, R);

    // Set initial state
    problem.init_state = x0;

    // Set the system
    problem.system = discrete_system;

    // Set the cost function
    problem.cost_function = cost_function;

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
    for (int i = 0; i < solution.states.size()-1; ++i) {
        // Write the current state to the file
        const auto& x = solution.states.at(i);
        const auto& u = solution.controls.at(i);
        file << t0 + i * dt << "," << x(0) << "," << x(1) << "," << x(2) << ","
             << x(3) << "," << u(0) << std::endl;
    }
    file.close();

    return 0;
}
