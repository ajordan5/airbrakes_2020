import numpy as np
import matplotlib.pyplot as plt
import sys
import rocketParam as P
from altitudePredictor import Predictor
from rocketDynamics import rocketDynamics
from rocketdataPlotter import dataPlotter
from rocketController import rocketController

"""Simulation of a rocket that flies vertically given an initial altitude and vertical speed. The final altitude
is controlled by an airbrakes system by matching a pre-determined reference velocity."""

# instantiate apogee predictor, dynamics, plotter and control
apogee = Predictor()
rocket = rocketDynamics()
plot = dataPlotter()
controller = rocketController()
h_history = []

# initialize rocket states
state = rocket.h()
h_dot = P.hdot0
h = state.item(0)
h_d1 = state.item(0)  # Altimeter reading delayed by one time step

# Numerical derivative to calculate vertical velocity from altimeter readings
def dirty_deriv(h, h_d1, h_dot):
    h_dot = P.beta * h_dot + (1 - P.beta) * ((h - h_d1) / P.Ts)

    return h_dot

t = P.t_start
while h_dot>=0:
    t_next_plot = t + P.t_plot

    # Update plot less frequent than simulation
    while t<t_next_plot:
        h_d1 = h  # save the previous h for dirty derivative
        h = state.item(0)
        h_dot = dirty_deriv(h, h_d1, h_dot)
        h_dot_ref = P.h_dot_ref_func3(h)

        # update control gain
        controller.gain(h, h_dot_ref)

        # update control output
        theta = controller.update(h_dot, h_dot_ref)
        # saturate theta
        theta = controller.saturate(theta, h, h_dot)
        state = rocket.update(theta)

        # Predict apogee based on no airbrakes
        if h_dot>=0:
            apogee_p = apogee.predict(h, h_dot)
        #print(apogee_p*3.28)
        h_history.append(h)

        t = t + P.Ts

    plot.update(t, h*3.28, h_dot, h_dot_ref, theta, apogee_p*3.28, P.apogee_ref)
    plt.pause(0.0001)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
apogee = max(h_history)*3.28  # Apogee converted to feet
print('apogee:', apogee)
print('error:', apogee-10000)
plt.waitforbuttonpress()
#plt.close()