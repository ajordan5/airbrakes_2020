import sys
sys.path.append('..')  # add parent directory
import numpy as np
import rocketParam as P
import random

class rocketDynamics:
    def __init__(self, alpha=0.1):
        # Initial state conditions
        self.state = np.array([
            [P.h0],      # altitude of motor burnout
            [P.hdot0],   # vertical speed
        ])  # initial angular rate

        #rocket parameters with random variation
        self.m = P.m * (1.+alpha*(2.*np.random.rand()-1.))
        self.Cd = P.Cd * (1.+alpha*(2.*np.random.rand()-1.))
        self.Cd_p = P.Cd_p * (1. + alpha * (2. * np.random.rand() - 1.))
        self.A = P.A #* (1.+alpha*(2.*np.random.rand()-1.))
        # self.rho = P.rho * (1.+alpha*(2.*np.random.rand()-1.))
        self.rho_var = (1. + alpha * (2. * np.random.rand() - 1.)) # rho variance
        self.rho = P.rho_func(P.h0) * self.rho_var
        self.Ts = P.Ts
        self.g = P.g
        self.alpha = alpha # noise param
        print(self.m, self.Cd, self.Cd_p, self.rho)

    def update(self, u):
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
        # saturate the input torque
        # u = self.saturate(u, self.force_limit)

        self.rk4_step(u)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output
        return y


    def f(self, state, u):
        # Return xdot = f(x,u), the system state update equations
        # re-label states for readability

        h = state.item(0)
        hdot = state.item(1)
        theta = u

        delta_A = P.A_func(theta)

        #vertical acceleration with a different Cd on the paddles
        rho = P.rho_func(h) * self.rho_var
        hddot = -self.g - ((self.Cd * self.A + self.Cd_p * delta_A) * rho * hdot ** 2) / (2 * self.m)
        xdot = np.array([[hdot], [hddot]])

        paddle_accel = self.Cd_p*delta_A*rho*hdot**2/(2 * self.m)

        ''''
        if paddle_accel > P.accel_limit*P.g:
            print("OVER")
            print(paddle_accel/P.g)
        '''

        return xdot


    def h(self):
        # return the output equations
        # could also use input u if needed
        h = self.state.item(0)
        hdot = self.state.item(1)

        return np.array([h, hdot])


    def rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2 * F1, u)
        F3 = self.f(self.state + self.Ts / 2 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)