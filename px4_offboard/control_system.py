import numpy as np
import math
from scipy.optimize import minimize
import casadi as ca


class ControlSystem:
    def __init__(self, prediction_horizon, timestep, initial_state, max_velocity, K_matrix):
        self.N = prediction_horizon
        self.dt = timestep
        self.state = ca.MX(initial_state)
        self.K = ca.MX(K_matrix)
        # self.Q = ca.MX.eye(4)
        # self.R = ca.MX.eye(4)
        # [5.48169041 0.51849889 9.59324074 3.17983114 9.03843909 5.40182631, 2.21062959 7.90420299]
        self.Q = ca.diag([2.1212808,3.16755349,7.9567266,0.83231977])
        self.R = ca.diag([8.35439349,1.16593257,3.74373557,3.15607747])
        # self.Q = ca.diag([1,1,1,1])
        # self.R = ca.diag([0.1,0.1,0.1,0.1])
        self.u_bounds = [(-max_velocity, max_velocity)] * 3 + [(-ca.pi/6, ca.pi/6)]
        self.target_feature = ca.MX([0, 0, 0, ca.pi/2])

    def set_K_matrix(self, K_matrix):
        self.K = ca.MX(K_matrix)

    def update_state(self, new_state):
        self.state = ca.MX(new_state)

    def compute_feature_vector(self, state):
        Y = 8524.24 / state[0] - 1.37
        Y = ca.if_else(state[0] != 0, Y, 0)  # Avoid division by zero using CasADi's if_else
        X = -state[1] * Y / self.K[0, 0]
        X = ca.if_else(self.K[0, 0] != 0, X, 0)  # Avoid division by zero using CasADi's if_else
        Z = state[2] * Y / self.K[1, 1]
        Z = ca.if_else(self.K[1, 1] != 0, Z, 0)  # Avoid division by zero using CasADi's if_else
        epsilon = 1e-6
        yaw = ca.atan2(Y + epsilon, X + epsilon)
        return ca.vertcat(X, Y, Z, yaw)

    def compute_dynamics(self, state, feature_vector):
        g = ca.MX.zeros((3, 4))
        g[0, 1] = (state[0]**2) / 8524.24
        g[0, 3] = (state[0]**2) * feature_vector[0] / 8524.24
        nonzero_condition = feature_vector[1] != 0
        g[1, 0] = ca.if_else(nonzero_condition, self.K[0, 0] / feature_vector[1], 0)
        g[1, 1] = ca.if_else(nonzero_condition, -self.K[0, 0] * feature_vector[0] / (feature_vector[1]**2), 0)
        g[1, 3] = ca.if_else(nonzero_condition, -self.K[0, 0] * (feature_vector[0]**2 + feature_vector[1]**2) / (feature_vector[1]**2), 0)
        g[2, 1] = ca.if_else(nonzero_condition, (self.K[1, 1] * feature_vector[2]) / (feature_vector[1]**2), 0)
        g[2, 2] = ca.if_else(nonzero_condition, -self.K[1, 1] / feature_vector[1], 0)
        g[2, 3] = ca.if_else(nonzero_condition, (self.K[1, 1] * feature_vector[0] * feature_vector[2]) / (feature_vector[1]**2), 0)

        # g[3, 0] = -feature_vector[1] / (feature_vector[0]**2 + feature_vector[1]**2 + 1e-6)  # Avoid division by zero
        # g[3, 1] = feature_vector[0] / (feature_vector[0]**2 + feature_vector[1]**2 + 1e-6)
        # g[3, 2] = 0
        # g[3, 3] = 1  # Direct yaw input control

        return g

    def next_state(self, state, control_input):
        feature_vector = self.compute_feature_vector(state)
        dS = ca.mtimes(self.compute_dynamics(state, feature_vector), control_input) * self.dt
        return state + dS

    def compute_cost(self, state, control_input):
        feature_vector = self.compute_feature_vector(state)
        state_error = feature_vector - self.target_feature
        state_cost = ca.mtimes(state_error.T, ca.mtimes(self.Q, state_error))  # Ensure proper multiplication order
        control_cost = ca.mtimes(control_input.T, ca.mtimes(self.R, control_input))  # Ensure proper multiplication order

        return state_cost + control_cost

    def compute_initial_guess(self,state):
        return (self.compute_feature_vector(state) - self.target_feature)

    def optimise_trajectory(self, initial_guess=None):
        u = ca.MX.sym('u', self.N, 4)
        state = self.state
        cost = 0

        for i in range(self.N):
            control_input = u[i, :].T  # Transpose to match dimensions
            cost += self.compute_cost(state, control_input)
            state = self.next_state(state, control_input)

        nlp = {'x': ca.reshape(u, -1, 1), 'f': cost}
        # solver = ca.nlpsol('solver', 'ipopt', nlp, {'ipopt.print_level': 0})
        # u0 = ca.DM.zeros((self.N * 4, 1))  # Adjust initial guess shape to match reshaped 'u'

        solver_options = {
            'ipopt': {
                'print_level': 0,          # Suppress IPOPT console output
                'max_cpu_time': 0.1,       # Limit solver to 1 second of CPU time
                'max_iter': 500,           # Limit to 500 iterations
                'tol': 1e-3,               # Set a looser tolerance if needed
                'acceptable_tol': 1e-2,    # Acceptable tolerance for early termination
                'acceptable_iter': 10      # Allow early stopping after 10 acceptable iterations
            }
            # ,
            # 'print_time': False,
            # 'verbose': False
        }

        solver = ca.nlpsol('solver', 'ipopt', nlp, solver_options)

        if initial_guess is None:
            u0 = ca.repmat(self.compute_initial_guess(self.state), self.N, 1)
        else:
            u0 = initial_guess

        result = solver(x0=u0.reshape((self.N*4,1)), 
                      lbx=[bound[0] for bound in self.u_bounds] * self.N, 
                      ubx=[bound[1] for bound in self.u_bounds] * self.N)

        return result

    def get_optimal_trajectory(self, initial_guess=None):
        result = self.optimise_trajectory(initial_guess)
        if result is not None:
            optimal_trajectory = ca.reshape(result['x'], self.N, 4)
            
            # Calculate and print the final cost - using evalf() to evaluate the symbolic expression
            # final_cost = float(ca.evalf(result['f']))
            # print("Debug: Before printing optimization cost")
            # print(f"Optimization Cost: {final_cost:.4f}")
            
            return True, ca.evalf(optimal_trajectory).full()
        else:
            print("Optimization failed")
            return False, None
