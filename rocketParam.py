import numpy as np

# Rocket and environmental parameters
m = 21.77           # Mass in kg
g = 9.81
Cd = .2989          # Cd of rocket with no paddles
Cd_p = 1.1          # Cd on the paddles. Flat plate = 1.17-1.28. Circular disk = 1.1
A = .0182           # Cross section of rocket in m^2
rho = .881          # Approximate air density at launch altitude (kg/m3)
gamma = rho*A*Cd/2  # Constant from original EoM

# initial conditions
h0 = 550.0
hdot0 = 290.681

# derivative of paddle area with respect to theta
Ap_prime = 0.0003
K_paddle = 0.0000838708 # coeff for area model

# Simulation Parameters
t_start = 0.0       # Start time of simulation
t_end = 100.0       # End time of simulation
Ts = 0.01           # sample time for simulation
t_plot = 0.5        # the plotting and animation is updated at this rate
Ts_predict = .2
Tf = 100            # simulation time
apogee_ref = 10000  # reference final apogee

# dirty derivative parameters
sigma = 0.005  # cutoff freq for dirty derivative
beta = (2.0*sigma-Ts)/(2.0*sigma+Ts)  # dirty derivative gain

# Transfer function constants
b1 = -rho*hdot0*A*Cd/m
b2 = -rho*hdot0**2*Ap_prime*Cd_p/(2*m)

# Saturation limit for paddle deployment percentage
limit = 60
accel_limit = 1.5 # gs

# gain calculation tuning parameters
tr = 6.50    #rise time
wn = 2.2/tr
kp = (wn-b1)/b2

print("kp:", kp)

# 9th order Polynomial fit to desired trajectory. Produces a desired velocity for a given altitude. Cd = .275
h_dot_ref_func = np.poly1d([-2.43906995324802e-27, 3.61890337922907e-23,
                            -2.33163379912629e-19, 8.53775438931857e-16,
                            -1.95321871332176e-12, 2.88785841484064e-09,
                            -2.75407922982037e-06, 0.00162916514619077,
                            -0.609031527035327, 371.484442576275])

# Polynomial for trajectory with over predicted Cd = .2989*1.1
h_dot_ref_func2 = np.poly1d([-2.45440390906831e-27, 3.64325936798749e-23,
                            -2.34839090100954e-19, 8.60325833301395e-16,
                            -1.96922475898631e-12, 2.91315564694316e-09,
                            -2.77990051814265e-06, 0.00164656382968312,
                            -0.621737107365176,383.975474579187])

# Polynomial for trajectory with variable density, ground temp = 5 C
h_dot_ref_func3 = np.poly1d([-1.08787581480336e-26,	1.68768550943230e-22,
                             -1.13390433997795e-18,	4.31975934256305e-15,
                             -1.02581964050172e-11,	1.57081214776205e-08,
                             -1.54745040884816e-05,	0.00943743006371261,
                             -3.30335627402355,	780.092640161965])

# Area model. Function for area as theta increases
A_func = np.poly1d([-1e-6, .0003, .0007])

# density function
t_ground = 6.03837
def rho_func(h):
    rho = 4.174794718e-11*\
          ((t_ground +273.1)
           - .001978152*h)**4.256

    T = t_ground - .00649*h
    P = 101.29 * ((T+273.1)/288.08)**5.256
    rho = P/(.2869*(T+273.1))
    return rho

