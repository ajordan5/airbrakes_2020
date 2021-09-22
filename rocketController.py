import rocketParam as P
import numpy as np


class rocketController:
    """A proportional controller that returns a paddle angle
    by conducting control on the error = velocity_ref - velocity.
    """

    def __init__(self):
        self.kp = P.kp
        self.limit = P.limit

    def update(self, v, vr):
        error = vr - v

        """ Update kp because kp was calculated based on an ideal 
        velocity which changes."""

        # self.gain(h, vr)
        theta = error * self.kp
        # theta_sat = self.saturate(theta)

        return theta

    def saturate(self, theta, h, hdot):

        if theta > self.limit:
            theta = self.limit
        elif theta < 0:
            theta = 0

        delta_A = P.A_func(theta)
        rho = P.rho_func(h)
        paddle_accel = P.Cd_p * delta_A * rho * hdot ** 2 / (2 * P.m)

        # saturate theta for max acceleration
        # print(paddle_accel/P.g)
        if paddle_accel > P.accel_limit*P.g:
            a1 = 2*P.m*P.accel_limit*P.g/(P.Cd_p*rho*hdot**2)
            theta = (a1-.0007)/.0003
            # print(P.Cd_p * P.A_func(theta) * rho * hdot ** 2 / (2 * P.m * P.g))

        return theta

    def gain(self, h, hdot0):
        # input current h and hdot ref

        # tf constants
        b1 = -P.rho_func(h) * hdot0 * P.A * P.Cd / P.m
        b2 = -P.rho_func(h) * hdot0 ** 2 * P.Ap_prime * P.Cd_p / (2 * P.m)

        # tuning parameter: rise time
        tr = P.tr
        wn = 2.2 / tr

        self.kp = (wn - b1) / b2
