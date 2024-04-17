#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import pandas as pd

# Read the output.csv file into a pandas dataframe
df = pd.read_csv("output.csv")

# Extract the time and state data
t = df["t"].values      # time
x = df["q1"].values     # cart position
q = df["q2"].values     # pendulum angle
v = df["dq1"].values    # cart velocity
w = df["dq2"].values    # pendulum angular velocity
u = df["u"].values      # control input
dt = t[1]-t[0]          # time step

# Create a 2x2 grid of plots
grid  = plt.GridSpec(3, 2, wspace=0.5, hspace=0.5)

# Plot the change of the cart position
plt.subplot(grid[0, 0])
plt.plot(t, x)
plt.title('Cart Position')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.grid(True)

# Plot the change of the pendulum angle
plt.subplot(grid[0, 1])
plt.plot(t, q)
plt.title('Pendulum Angle')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.grid(True)

# Plot the change of the cart velocity
plt.subplot(grid[1, 0])
plt.plot(t, v)
plt.title('Cart Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.grid(True)

# Plot the change of the pendulum angular velocity
plt.subplot(grid[1, 1])
plt.plot(t, w)
plt.title('Pendulum Angular Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.grid(True)

# Plot the change of the control input
plt.subplot(grid[2, :])
plt.step(t, u, where='post')
plt.title('Control Input')
plt.xlabel('Time (s)')
plt.ylabel('Force (N)')
plt.grid(True)

# Adjust the layout
plt.tight_layout()

# Set the widh and height of the figure
plt.gcf().set_size_inches(12, 8)

# Save the plots to a file
plt.savefig("pendulum_on_cart.png")

# Display the plots
plt.show()
