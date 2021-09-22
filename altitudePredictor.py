import numpy as np
import sys
import rocketParam as P

sys.path.append('..')

class Predictor:
    def __init__(self):
        self.m = P.m
        self.Cd = P.Cd
        self.A = P.A
        self.rho = P.rho
        self.sig = .05          #sigma for dirty deriv
        self.hdot_prev = 0
        self.h_prev = 0         #saves initial h for first h prev
        self.Ts = P.Ts_predict
        self.g = P.g
        self.t_final = 25



    def predict(self, h, hdot):
        self.altitude_history = [0, .001]  # these two values allow us to start the while loop below
        #hdot is done in the main file and plugged into the predictor
        #hdot = ((2*self.sig-self.Ts)/(2*self.sig+self.Ts))*self.hdot_prev + (2/(2*self.sig+self.Ts)) * (h - self.h_prev)

        self.state = np.array([[h], [hdot]])

        t = 0
        i = 0

        #solve ODE into future until the rocket starts falling back to earth
        while self.altitude_history[i+1]>self.altitude_history[i]:


            #rk4 to solve ODE for next time step.
            F1 = self.f(self.state)
            F2 = self.f(self.state + self.Ts / 2 * F1)
            F3 = self.f(self.state + self.Ts / 2 * F2)
            F4 = self.f(self.state + self.Ts * F3)

            # This is what state is predicted to be on the next time step
            self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)


            '''
            #Euler method
            F1 = self.f(self.state + self.Ts)

            self.state += F1*self.Ts
            '''
            #add predicted apogee to history
            self.altitude_history.append(self.state.item(0))

            #increment counter and time step
            i= i+1
            t = t + self.Ts
        
        return max(self.altitude_history)





    def f(self, state):
        hdot = state.item(1)
        hddot = -self.g - (self.Cd * self.rho * self.A * hdot ** 2) / (2 * self.m)

        xdot = np.array([[hdot], [hddot]])

        return xdot

