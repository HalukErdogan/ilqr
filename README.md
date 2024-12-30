# ILQR - Iterative Linear Quadratic Regulator

**Iterative Linear Quadratic Regulator (iLQR)** is an optimal control algorithm commonly used to solve trajectory optimization problems for nonlinear systems. This repository offers a pure C++ implementation that prioritizes speed, minimal dependencies, and straightforward integration. The corresponding optimal control problem can be formally stated as follows:

```math
\begin{aligned}
 \min_{\mathbf{x}, \mathbf{u}} \quad & \bigg[ J(\mathbf{x}, \mathbf{u}) := l_f(x_H) + \sum_{i=0}^{H-1} l(x_i, u_i)  \bigg] \\
 \\
\text{subject to} \quad & x_{i+1} = f(x_i, u_i), 
\\
& x_0 = x_{init}, \\
& g_i(\mathbf{x}, \mathbf{u}) \leq 0, \quad \forall i \in \{1,2,\dots,K\}, \\
& x_{lower} \leq x_i \leq x_{upper}, \quad \forall i \in \{1,2,\dots,H\}, \\
& u_{lower} \leq u_i \leq u_{upper}, \quad \forall i \in \{1,2,\dots,H-1\}, \\
\\
\text{where} \quad & \mathbf{x} = \{x_0, x_1,\dots, x_H \}, \\
& \mathbf{u} = \{u_0, u_1, \dots, u_{H-1}\}, \\
& x_i \in \mathbb{R}^N \quad \forall i \in \{1,2,\dots,H\}, \\
& u_i \in \mathbb{R}^M \quad \forall i \in \{1,2,\dots,H-1\}, \\
& x_{init}, x_{lower}, x_{upper} \in \mathbb{R}^N, \\
& u_{lower}, u_{upper} \in \mathbb{R}^M.
\end{aligned}
```

Whether you're a researcher prototyping new algorithms or an engineer requiring real-time solutions, this library simplifies the setup and execution of iLQR optimizations.
- **Pure C++ Implementation:** – Written entirely in C++, eliminating external overhead and language bindings.
- **Minimal Dependencies:** – Requires only Eigen, with optional GTest for testing.
- **Easy Dependency Management:** –  Manage and install seamlessly via vcpkg.
- **Cross-Platform:** – Verified on Ubuntu 22.04, Ubuntu 20.04, and Windows 11.
- **High Performance:** – Achieves zero dynamic memory allocations through templating.
- **Flexible Discretization:** – Offers multiple integration steppers for more precise control.
- **Numerical Differentiation:** – Supports both continuous and discrete systems, as well as cost functions.

## Requirements

Before installing ILQR, ensure you meet the following system requirements:

- Git
- CMake
- A build system such as Make or Ninja
- A C++ compiler like g++ or clang
- Optional: Matplotlib, Pandas, and Pillow for visualization

## Installation

Follow these steps to install ILQR:

1. Clone the repository:
    ```bash
    git clone https://github.com/HalukErdogan/ilqr.git
    ```

2. Navigate to the repository directory:
    ```bash
    cd ilqr
    ```

3. Initilize the submodule
    ```bash
    git submodule init
    ```
4. Update the submodule
    ```bash
    git submodule update
    ```

5. Configure the package:
    ```bash
    cmake --preset default
    ```

6. Build the package:
 
    ```bash
   cmake --build --preset default
    ```

## Example

### Pendulum on Cart
To showcase the library's capabilities, the classical problem known as the pendulum on a cart is solved using the library. The optimal control problem is formulated as follows

```math
\begin{aligned}
 \min_{\mathbf{x}, \mathbf{u}} \quad & \left[ J(\mathbf{x}, \mathbf{u}) := (x_{target} - x_H)^T Q_f (x_{target} - x_H)  + \sum_{i=0}^{H-1} \left[(x_{target} -x_i)^T  Q  (x_{target} -x_i) + u_i^T R u_i \right] \right] \\
 \\
\text{subject to} \quad & \\
& \dot{x} = f(x, u) = 
    \begin{bmatrix}
        \dot{q_1} \\
        \dot{q_2} \\
        \dot{q_3} \\
        \dot{q_4}
    \end{bmatrix} =
    \begin{bmatrix} 
		q_3 \\
        q_4 \\
		\frac{u + l \, m_2 \, sin(q_2) \, q_4^2 + m_2 \, g \, sin(q_2) \, cos(q_2) }{m_1 + m_2 \, sin{q_2}^2 } \\
        - \frac{u \, cos(q_2) + l \, m_2 \, sin(q_2) \, cos(q_2) \, q_4^2 +  (m_1 + m_2) \, g \,sin(q_2)}{l \, (m_1 + m_2 \, sin{q_2}^2 )}
	\end{bmatrix}, 
\\
& x_0 = x_{init}, \\
& x_{lower} \leq x_i \leq x_{upper}, \quad \forall i \in \{1,2,\dots,H\}, \\
& u_{lower} \leq u_i \leq u_{upper}, \quad \forall i \in \{1,2,\dots,H-1\}, \\
\\
\text{where} \quad & \\
& \mathbf{x} = \{x_0, x_1,\dots, x_H \}, \\
& \mathbf{u} = \{u_0, u_1, \dots, u_{H-1}\}, \\
& x_i \in \mathbb{R}^N \quad \forall i \in \{1,2,\dots,H\}, \\
& u_i \in \mathbb{R}^M \quad \forall i \in \{1,2,\dots,H-1\}, \\
& x_{init}, x_{lower}, x_{upper} \in \mathbb{R}^N, \\
& u_{lower}, u_{upper} \in \mathbb{R}^M.
\end{aligned}
```

The problem is solved by discretizing the continuous dynamics. Check the 'example/pendulum_on_cart' folder for more details. Follow the steps provided below to execute the example:

1. Run the executable:

    For Windows:
    ```bash
    ./build/examples/pendulum_on_cart/Release/pendulum_on_cart.exe
    ```
    For Linux:
    ```bash
    ./build/examples/pendulum_on_cart/pendulum_on_cart
    ```
2. Visualize the results:
    ```bash
    python3 ./examples/pendulum_on_cart/scripts/visualize.py
    ```
   Result:

   ![](examples/pendulum_on_cart/plot/pendulum_on_cart.png)

3. Animate the results:
    ```bash
    python3 ./examples/pendulum_on_cart/scripts/animate.py
    ```
   
   Result:

   ![](examples/pendulum_on_cart/gif/pendulum_on_cart.gif)

## Performance

The performance of the module is tested using the pendulum on cart example provided above. The following table show the execution time of the algorithm in seconds. Tests has been done on 12th Gen Intel(R) Core(TM) i7-12800H processor and Ubuntu 22.04 WSL operation system. The example is built in Release configuration using g++ and make.


|                          | Horizon = 101 | Horizon = 501 | Horizon = 1001|
|--------------------------|---------------|---------------|---------------|
| Euler Integration        | 0.00297155 s  | 0.00924278 s  | 0.0175842 s   |
| Runge Kutta 2nd Order    | 0.00415075 s  | 0.0180604 s   | 0.0357379 s   |
| Runge Kutta 3nd Order    | 0.00721841 s  | 0.0264217 s   | 0.0532759 s   |
| Runge Kutta 4nd Order    | 0.00782524 s  | 0.0363073 s   | 0.0706778 s   |

Note: I wasn't able to recreate same performance results with MSVC. The results were up to 5 times slower with MSVC (Tested on same PC).

## Roadmap

- [x] Add continuous system base class
- [x] Add discrete system base class
- [x] Add cost function base class
- [x] Add continuous system with finite diff class
- [x] Add discrete system with finite diff class
- [x] Add cost function with finite diff class
- [x] Add test for finite diff classes
- [x] Add Euler integration stepper
- [x] Add second order Runge - Kutta integration stepper
- [x] Add third order Runge - Kutta integration stepper
- [x] Add fourth order Runge - Kutta integration stepper
- [x] Add constant integration iterator
- [x] Add discritizer class
- [x] Add quadratic cost function
- [x] Add ilqr class
- [x] Add line search
- [x] Add example: pendulum on cart
- [x] Add scripts to visualize and animate the result of example
- [x] Add inequality constrains using control barrier functions
- [x] Add box constraints for state and control variables
- [ ] Add adaptive integration iterator

## Contact

Author: Haluk Erdogan

Email: haluk_erdogan@outlook.com

Linkedin: [haluk_erdogan](https://www.linkedin.com/in/halukerdogan/)
