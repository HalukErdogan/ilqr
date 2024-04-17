#ifndef ILQR_CONTROLLER_ILQR_HPP
#define ILQR_CONTROLLER_ILQR_HPP

#include <Eigen/Dense>
#include <array>
#include <iostream>
#include <memory>

#include "ilqr/cost_functions/cost_function.hpp"
#include "ilqr/systems/discrete_time/discrete_system.hpp"

namespace ilqr {
namespace controller {

using ilqr::cost_functions::CostFunction;
using ilqr::systems::discrete_time::DiscreteSystem;

template <std::size_t N, std::size_t M>
struct Problem {
    Eigen::Matrix<double, N, 1> init_state;
    std::shared_ptr<DiscreteSystem<N, M>> system;
    std::shared_ptr<CostFunction<N, M>> cost_function;
};

struct Options {
    std::size_t max_iter;
    double tolerance = 1e-6;
    double regularization = 1e-6;
    bool warm_start = false;
};

template <std::size_t N, std::size_t M, std::size_t H>
struct Solution {
    std::array<Eigen::Matrix<double, N, 1>, H> states;
    std::array<Eigen::Matrix<double, M, 1>, H - 1> controls;
    std::array<Eigen::Matrix<double, M, 1>, H - 1> feedforward_gains;
    std::array<Eigen::Matrix<double, M, N>, H - 1> feedback_gains;
};

template <std::size_t N, std::size_t M, std::size_t H>
class ILQR {
   public:
    void Optimize(const Problem<N, M>& problem, const Options& options,
                  Solution<N, M, H>& solution) {
        // Simulate forward
        solution.states[0] = problem.init_state;

        // Reset the control inputs
        if (options.warm_start == false) {
            for (std::size_t j = 0; j < H - 1; ++j) {
                solution.controls[j] = Eigen::Matrix<double, M, 1>::Zero();
            }
        } 

        // Rollout the system
        double cost = 0.0;
        for (std::size_t j = 0; j < H - 1; ++j) {
            const auto& x = solution.states[j];
            const auto& u = solution.controls[j];
            auto& x_next = solution.states[j + 1];

            // Compute the dynamics
            problem.system->PropogateDynamics(x, u, j, x_next);

            // Compute the running cost
            cost += problem.cost_function->CalculateRunningCost(x, u, j);
        }

        // Compute the terminal cost
        cost += problem.cost_function->CalculateTerminalCost(
            solution.states[H - 1], H - 1);

        // Store the previous cost
        double prev_cost = cost;

        // Log the initial cost
        std::cout << "Initial cost: " << cost << std::endl;

        // Iterate until convergence
        for (std::size_t i = 0; i < options.max_iter; ++i) {
            // Backward pass
            problem.cost_function->CalculateTerminalCostStateGradient(
                solution.states[H - 1], H - 1, Vx);
            problem.cost_function->CalculateTerminalCostStateHessian(
                solution.states[H - 1], H - 1, Vxx);
            for (int j = H - 2; j >= 0; --j) {
                const auto& x = solution.states[j];
                const auto& u = solution.controls[j];
                auto& k = solution.feedforward_gains[j];
                auto& K = solution.feedback_gains[j];

                // Compute the dynamics derivatives
                problem.system->CalculateStateJacobian(x, u, j, Fx);
                problem.system->CalculateControlJacobian(x, u, j, Fu);

                // Compute the cost function derivatives
                problem.cost_function->CalculateRunningCostStateGradient(x, u, j, Lx);
                problem.cost_function->CalculateRunningCostControlGradient(x, u, j, Lu);
                problem.cost_function->CalculateRunningCostStateHessian(x, u, j, Lxx);
                problem.cost_function->CalculateRunningCostControlStateHessian(x, u, j, Lux);
                problem.cost_function->CalculateRunningCostControlHessian(x, u, j, Luu);

                // Compute the Q function derivatives
                Qx = Lx + Fx.transpose() * Vx;
                Qu = Lu + Fu.transpose() * Vx;
                Qxx = Lxx + Fx.transpose() * Vxx * Fx;
                Qux = Lux + Fu.transpose() * Vxx * Fx;
                Quu = Luu + Fu.transpose() * Vxx * Fu;
                Quu_inv = (Quu + options.regularization *
                                     Eigen::Matrix<double, M, M>::Identity())
                              .inverse();
                
                // Compute the feedforward and feedback gains 
                k = -Quu_inv * Qu;
                K = -Quu_inv * Qux;

                // Compute the value function derivatives
                Vx = Qx + K.transpose() * Qu + Qux.transpose() * k + K.transpose() * Quu * k;
                Vxx = Qxx + K.transpose() * Qux + Qux.transpose() * K + K.transpose() * Quu * K;
            }

            // Forward pass
            double alpha = 1.0;
            while (alpha > 1e-8) {
                double new_cost = 0.0;
                X_hat[0] = solution.states[0];
                for (std::size_t j = 0; j < H - 1; ++j) {
                    const auto& x = solution.states[j];
                    const auto& u = solution.controls[j];
                    const auto& k = solution.feedforward_gains[j];
                    const auto& K = solution.feedback_gains[j];
                    const auto& x_hat = X_hat[j];
                    auto& u_hat = U_hat[j];
                    auto& x_hat_next = X_hat[j + 1];

                    // Compute the control
                    u_hat = u + alpha * k + K * (x_hat - x);

                    // Simulate the dynamics
                    problem.system->PropogateDynamics(x_hat, u_hat, j,
                                                      x_hat_next);

                    // Compute the running cost
                    new_cost += problem.cost_function->CalculateRunningCost(
                        x_hat, u_hat, j);
                }

                // Compute the terminal cost
                new_cost += problem.cost_function->CalculateTerminalCost(
                    X_hat[H - 1], H - 1);

                // Check if the new cost is less than the previous cost
                if (new_cost < cost) {
                    cost = new_cost;
                    break;
                }

                // Decrease the step size
                alpha *= 0.5;
            }

            // Update the states and controls
            std::swap(solution.states, X_hat);
            std::swap(solution.controls, U_hat);

            // Log the cost
            std::printf("Iteration: %5zu, Alpha: %.5f, Cost: %.5f\n", i, alpha, cost);

            // Check for convergence
            if (std::abs(prev_cost - cost) < options.tolerance) {
                // Log the final cost
                std::cout << "Algorithm converged! Final cost: " << cost << std::endl;
                return;
            }

            // Update the previous cost
            prev_cost = cost;
        }

        // Log the final cost
        std::cout << "Algorithm reached iteration limit! Final cost: " << cost << std::endl;
    }

   private:
    // dynamic function diff
    Eigen::Matrix<double, N, N> Fx;
    Eigen::Matrix<double, N, M> Fu;

    // cost function diff
    double L;
    Eigen::Matrix<double, N, 1> Lx;
    Eigen::Matrix<double, M, 1> Lu;
    Eigen::Matrix<double, N, N> Lxx;
    Eigen::Matrix<double, M, N> Lux;
    Eigen::Matrix<double, M, M> Luu;

    // value function diff
    double V;
    Eigen::Matrix<double, N, 1> Vx;
    Eigen::Matrix<double, N, N> Vxx;

    // Q function diff
    Eigen::Matrix<double, N, 1> Qx;
    Eigen::Matrix<double, M, 1> Qu;
    Eigen::Matrix<double, N, N> Qxx;
    Eigen::Matrix<double, M, N> Qux;
    Eigen::Matrix<double, M, M> Quu;
    Eigen::Matrix<double, M, M> Quu_inv;

    // Buffer
    std::array<Eigen::Matrix<double, N, 1>, H> X_hat;
    std::array<Eigen::Matrix<double, M, 1>, H - 1> U_hat;
};

}  // namespace controller
}  // namespace ilqr

#endif  // ILQR_CONTROLLER_ILQR_HPP