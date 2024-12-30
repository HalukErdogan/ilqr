#! /usr/bin/env python3
# -- coding: utf-8 --

import math
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Constants
CART_WIDTH = 0.5
CART_HEIGHT = 0.3
PENDULUM_LENGTH = 1

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

# Create a new figure and set the aspect ratio
fig, ax = plt.subplots()

# Plot the cart as a cart
cart = plt.Rectangle((-CART_WIDTH/2, -CART_HEIGHT/2), CART_WIDTH, CART_HEIGHT, color='red')
ax.add_patch(cart)

# Plot the pendulum as a line
pendulum, = ax.plot([0, 0], [0, 0], '-o', lw=4, color='black')

# Plot the track as a line
track, = ax.plot([-100, 100], [0, 0], '--', lw=1, color='black')

# Add a text object for the time
time_text = ax.text(0.05, 0.95, '', transform=ax.transAxes, verticalalignment='top')

def init():
    # Reset the cart and pendulum positions
    cart.set_x(x[0] - CART_WIDTH/2)
    pendulum.set_xdata([x[0], x[0] + PENDULUM_LENGTH * math.sin(q[0])])
    pendulum.set_ydata([0, - PENDULUM_LENGTH * math.cos(q[0])])

    # Initialize the time text
    time_text.set_text(f'Time: {t[0]:.2f} s')

    # Set the title and labels
    ax.set_title('Pendulum on Cart')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')

    # Set the aspect ratio and limits
    ax.set_aspect('equal', 'box')
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)

    # Draw the initial plot
    fig.canvas.draw()

    return cart, pendulum, time_text

# Function to update the animation
def update(i):
    # Update the position of the cart and the pendulum based on the ith index
    cart.set_x(x[i] - CART_WIDTH/2)
    pendulum.set_xdata([x[i], x[i] + PENDULUM_LENGTH * math.sin(q[i])])
    pendulum.set_ydata([0, - PENDULUM_LENGTH * math.cos(q[i])])

    # Update the time display
    time_text.set_text(f'Time: {t[i]:.2f} s')
    
    return cart, pendulum, time_text

# Create the animation
ani = FuncAnimation(fig, update, frames=len(t+1), init_func=init, blit=True, interval=dt*1000)

# Save the animation as a gif
ani.save('pendulum_on_cart.gif', writer='pillow', fps=1/dt, dpi=100)

# Display the animation
plt.show()