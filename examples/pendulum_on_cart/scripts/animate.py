#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Constants
BOX_WIDTH = 0.5
BOX_HEIGHT = 0.3
STICK_LENGTH = 1

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

# Plot the cart as a rectangle
rect = plt.Rectangle((-BOX_WIDTH/2, -BOX_HEIGHT/2), BOX_WIDTH, BOX_HEIGHT, color='red')
ax.add_patch(rect)

# Plot the pendulum as a line
line, = ax.plot([0, 0], [0, 0], '-o', lw=4, color='black')

# Add a text object for the time
time_text = ax.text(0.05, 0.95, '', transform=ax.transAxes, verticalalignment='top')

def init():
    # Set the title and labels
    ax.set_title('Pendulum on Cart')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')

    # Set the aspect ratio
    ax.set_aspect('equal', 'box')
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    
    # Plot the track
    ax.plot([-100, 100], [0, 0], '--', lw=1, color='black')

    # Initialize the time text
    time_text.set_text('')

    return rect, line, time_text

# Function to update the animation
def update(i):
    # Update the position of the rectangle and the line based on the ith index
    rect.set_x(x[i] - BOX_WIDTH/2)
    line.set_xdata([x[i], x[i] + STICK_LENGTH * math.sin(q[i])])
    line.set_ydata([0, - STICK_LENGTH * math.cos(q[i])])

    # Update the time display
    time_text.set_text(f'Time: {t[i]:.2f} s')

    # Sleep for the time step
    time.sleep(dt)
    
    return rect, line, time_text

# Function to generate the frames
def frames():
    for i in range(len(t)):
        yield i

# Create the animation
ani = FuncAnimation(fig, update, frames=frames, init_func=init, blit=True, interval=10)

# Save the animation as a gif
ani.save('pendulum_on_cart.gif', fps=1/dt, dpi=300)

# Display the animation
plt.show()
