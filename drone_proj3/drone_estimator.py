import matplotlib.pyplot as plt
import numpy as np
import time
plt.rcParams['font.family'] = ['Arial']
plt.rcParams['font.size'] = 14


class Estimator:
    """A base class to represent an estimator.

    This module contains the basic elements of an estimator, on which the
    subsequent DeadReckoning, Kalman Filter, and Extended Kalman Filter classes
    will be based on. A plotting function is provided to visualize the
    estimation results in real time.

    Attributes:
    ----------
        u : list
            A list of system inputs, where, for the ith data point u[i],
            u[i][1] is the thrust of the quadrotor
            u[i][2] is right wheel rotational speed (rad/s).
        x : list
            A list of system states, where, for the ith data point x[i],
            x[i][0] is translational position in x (m),
            x[i][1] is translational position in z (m),
            x[i][2] is the bearing (rad) of the quadrotor
            x[i][3] is translational velocity in x (m/s),
            x[i][4] is translational velocity in z (m/s),
            x[i][5] is angular velocity (rad/s),
        y : list
            A list of system outputs, where, for the ith data point y[i],
            y[i][1] is distance to the landmark (m)
            y[i][2] is relative bearing (rad) w.r.t. the landmark
        x_hat : list
            A list of estimated system states. It should follow the same format
            as x.
        dt : float
            Update frequency of the estimator.
        fig : Figure
            matplotlib Figure for real-time plotting.
        axd : dict
            A dictionary of matplotlib Axis for real-time plotting.
        ln* : Line
            matplotlib Line object for ground truth states.
        ln_*_hat : Line
            matplotlib Line object for estimated states.
        canvas_title : str
            Title of the real-time plot, which is chosen to be estimator type.

    Notes
    ----------
        The landmark is positioned at (0, 5, 5).
    """
    # noinspection PyTypeChecker
    def __init__(self, is_noisy=False):
        self.u = []
        self.x = []
        self.y = []
        self.x_hat = []  # Your estimates go here!
        self.t = []
        self.fig, self.axd = plt.subplot_mosaic(
            [['xz', 'phi'],
             ['xz', 'x'],
             ['xz', 'z']], figsize=(20.0, 10.0))
        self.ln_xz, = self.axd['xz'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_xz_hat, = self.axd['xz'].plot([], 'o-c', label='Estimated')
        self.ln_phi, = self.axd['phi'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_phi_hat, = self.axd['phi'].plot([], 'o-c', label='Estimated')
        self.ln_x, = self.axd['x'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_x_hat, = self.axd['x'].plot([], 'o-c', label='Estimated')
        self.ln_z, = self.axd['z'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_z_hat, = self.axd['z'].plot([], 'o-c', label='Estimated')
        self.canvas_title = 'N/A'

        # Defined in dynamics.py for the dynamics model
        # m is the mass and J is the moment of inertia of the quadrotor 
        self.gr = 9.81 
        self.m = 0.92
        self.J = 0.0023
        # These are the X, Y, Z coordinates of the landmark
        self.landmark = (0, 5, 5)

        # This is a (N,12) where it's time, x, u, then y_obs 
        if is_noisy:
            with open('noisy_data.npy', 'rb') as f:
                self.data = np.load(f)
        else:
            with open('data.npy', 'rb') as f:
                self.data = np.load(f)

        self.dt = self.data[-1][0]/self.data.shape[0]


    def run(self):
        for i, data in enumerate(self.data):
            self.t.append(np.array(data[0]))
            self.x.append(np.array(data[1:7]))
            self.u.append(np.array(data[7:9]))
            self.y.append(np.array(data[9:12]))
            if i == 0:
                self.x_hat.append(self.x[-1])
            else:
                self.update(i)
        return self.x_hat

    def update(self, _):
        raise NotImplementedError

    def plot_init(self):
        self.axd['xz'].set_title(self.canvas_title)
        self.axd['xz'].set_xlabel('x (m)')
        self.axd['xz'].set_ylabel('z (m)')
        self.axd['xz'].set_aspect('equal', adjustable='box')
        self.axd['xz'].legend()
        self.axd['phi'].set_ylabel('phi (rad)')
        self.axd['phi'].set_xlabel('t (s)')
        self.axd['phi'].legend()
        self.axd['x'].set_ylabel('x (m)')
        self.axd['x'].set_xlabel('t (s)')
        self.axd['x'].legend()
        self.axd['z'].set_ylabel('z (m)')
        self.axd['z'].set_xlabel('t (s)')
        self.axd['z'].legend()
        plt.tight_layout()

    def plot_update(self, _):
        self.plot_xzline(self.ln_xz, self.x)
        self.plot_xzline(self.ln_xz_hat, self.x_hat)
        self.plot_philine(self.ln_phi, self.x)
        self.plot_philine(self.ln_phi_hat, self.x_hat)
        self.plot_xline(self.ln_x, self.x)
        self.plot_xline(self.ln_x_hat, self.x_hat)
        self.plot_zline(self.ln_z, self.x)
        self.plot_zline(self.ln_z_hat, self.x_hat)

    def plot_xzline(self, ln, data):
        if len(data):
            x = [d[0] for d in data]
            z = [d[1] for d in data]
            ln.set_data(x, z)
            self.resize_lim(self.axd['xz'], x, z)

    def plot_philine(self, ln, data):
        if len(data):
            t = self.t
            phi = [d[2] for d in data]
            ln.set_data(t, phi)
            self.resize_lim(self.axd['phi'], t, phi)

    def plot_xline(self, ln, data):
        if len(data):
            t = self.t
            x = [d[0] for d in data]
            ln.set_data(t, x)
            self.resize_lim(self.axd['x'], t, x)

    def plot_zline(self, ln, data):
        if len(data):
            t = self.t
            z = [d[1] for d in data]
            ln.set_data(t, z)
            self.resize_lim(self.axd['z'], t, z)

    # noinspection PyMethodMayBeStatic
    def resize_lim(self, ax, x, y):
        xlim = ax.get_xlim()
        ax.set_xlim([min(min(x) * 1.05, xlim[0]), max(max(x) * 1.05, xlim[1])])
        ylim = ax.get_ylim()
        ax.set_ylim([min(min(y) * 1.05, ylim[0]), max(max(y) * 1.05, ylim[1])])

class OracleObserver(Estimator):
    """Oracle observer which has access to the true state.

    This class is intended as a bare minimum example for you to understand how
    to work with the code.

    Example
    ----------
    To run the oracle observer:
        $ python drone_estimator_node.py --estimator oracle
    """
    def __init__(self, is_noisy=False):
        super().__init__(is_noisy)
        self.canvas_title = 'Oracle Observer'

    def update(self, _):
        self.x_hat.append(self.x[-1])


class DeadReckoning(Estimator):
    """Dead reckoning estimator.

    Your task is to implement the update method of this class using only the
    u attribute and x0. You will need to build a model of the unicycle model
    with the parameters provided to you in the lab doc. After building the
    model, use the provided inputs to estimate system state over time.

    The method should closely predict the state evolution if the system is
    free of noise. You may use this knowledge to verify your implementation.

    Example
    ----------
    To run dead reckoning:
        $ python drone_estimator_node.py --estimator dr
    """
    def __init__(self, is_noisy=False):
        super().__init__(is_noisy)
        self.canvas_title = 'Dead Reckoning'
        self.computation_time = []

    def accuracy(self, ): # RSME
        x_array = np.array(self.x)         # Shape: (num_samples, num_dimensions)
        xhat_array = np.array(self.x_hat)  # Same shape as x_array
        
        min_len = min(len(x_array), len(xhat_array))

        # Trim both arrays to the same minimum length
        x_array_trimmed = x_array[:min_len]
        xhat_array_trimmed = xhat_array[:min_len]

        # Compute the difference and squared difference
        diff = x_array_trimmed - xhat_array_trimmed
        squared_diff = diff ** 2
                
        # Take the mean across samples (rows), keep separate for each dimension
        mean_squared_diff = np.mean(squared_diff, axis=0)
        
        # Take the square root to get RMSE
        rmse = np.sqrt(mean_squared_diff)
        return rmse
   
    def update(self, _):
        start_time = time.time()

        if len(self.x_hat) > 0:
            # TODO: Your implementation goes here!
            # You may ONLY use self.u and self.x[0] for estimation
            # x_hat already has x[0] appended to it somewhere in the starter code, just added the else in the event that it is not
            # x[0] is [x, y, phi, x_dot, y_dot, phi_dot]

            T = len(self.u)
            t = 0

            f_first = lambda x: np.array([[0, 0],
                                          [0, 0],
                                          [0, 0],
                                          [-np.sin(x[2])/self.m, 0],
                                          [np.cos(x[2])/self.m, 0],
                                          [0, 1/self.J]])
            
            f_second = lambda u: np.array([[u[0]],
                                           [u[1]]])

            f = lambda x, u: np.array([[x[3]],
                                       [x[4]],
                                       [x[5]],
                                       [0],
                                       [-self.gr],
                                       [0]]) \
                                       + np.dot(f_first(x), f_second(u))

            g = lambda x, u: np.array([[x[0]],
                                       [x[1]],
                                       [x[2]],
                                       [x[3]],
                                       [x[4]],
                                       [x[5]]]) \
                                       + f(x, u) * self.dt
            
            while t < T - 1:
                if len(self.x_hat) - 1 < t + 1:
                    self.x_hat.append(g(self.x_hat[t], self.u[t]).reshape(6))
                else: self.x_hat[t + 1] = g(self.x_hat[t], self.u[t]).reshape(6)
                t = t + 1
        else: 
            self.x_hat[0] = self.x[0]

        end_time = time.time()
        elapsed_time = end_time - start_time
        self.computation_time.append(elapsed_time)
        print(f"Average Execution time per step: {sum(self.computation_time)/len(self.computation_time)} seconds")
        print(f"Execution time: {elapsed_time} seconds")
        print("Accuracy: ", self.accuracy())

# noinspection PyPep8Naming
class ExtendedKalmanFilter(Estimator):
    """Extended Kalman filter estimator.

    Your task is to implement the update method of this class using the u
    attribute, y attribute, and x0. You will need to build a model of the
    unicycle model and linearize it at every operating point. After building the
    model, use the provided inputs and outputs to estimate system state over
    time via the recursive extended Kalman filter update rule.

    Hint: You may want to reuse your code from DeadReckoning class and
    KalmanFilter class.

    Attributes:
    ----------
        landmark : tuple
            A tuple of the coordinates of the landmark.
            landmark[0] is the x coordinate.
            landmark[1] is the y coordinate.
            landmark[2] is the z coordinate.

    Example
    ----------
    To run the extended Kalman filter:
        $ python drone_estimator_node.py --estimator ekf
    """
    def __init__(self, is_noisy=False):
        super().__init__(is_noisy)
        self.canvas_title = 'Extended Kalman Filter'
        # You may define the Q, R, and P matrices below.
        self.A = np.eye(4)

        # Measurement matrix: 2x4 (measuring position only)
        self.C = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        # Covariance of process noise: 4x4
        self.Q = .001 * np.eye(6)
        # Covariance of measurement noise: 2x2
        self.R = 0.1 * np.eye(2)
        # Initial covariance: 4x4
        self.P_0 = 1000 * np.eye(6)
        self.computation_time = []


    def accuracy(self, ): # RSME
        x_array = np.array(self.x)         # Shape: (num_samples, num_dimensions)
        xhat_array = np.array(self.x_hat)  # Same shape as x_array
        
        min_len = min(len(x_array), len(xhat_array))

        # Trim both arrays to the same minimum length
        x_array_trimmed = x_array[:min_len]
        xhat_array_trimmed = xhat_array[:min_len]

        # Compute the difference and squared difference
        diff = x_array_trimmed - xhat_array_trimmed
        squared_diff = diff ** 2
                
        # Take the mean across samples (rows), keep separate for each dimension
        mean_squared_diff = np.mean(squared_diff, axis=0)
        
        # Take the square root to get RMSE
        rmse = np.sqrt(mean_squared_diff)
        return rmse
   

    # noinspection DuplicatedCode
    def update(self, i):
        start_time = time.time()

        if len(self.x_hat) > 0: #and self.x_hat[-1][0] < self.x[-1][0]:
            t = 0
            self.x_hat[t] = self.x[0]
            P = [self.P_0.copy()]
            K = []
            T = len(self.u)
            while t < T:
                # State Extrapolation
                next_x_hat_given_t = self.g(self.x_hat[t], self.u[t])

                A_next = self.approx_A(self.x_hat[t], self.u[t])

                next_P_given_t = A_next @ (P[t]) @ A_next.T + self.Q
                
                C_next = self.h_prime(next_x_hat_given_t) # h_prime[x,y] values

                k_gain = np.linalg.inv(C_next @ next_P_given_t @ C_next.T + self.R)
                K.append(next_P_given_t @ C_next.T @ k_gain)
                # print("y : ", self.y)

                # State Update

                # print("y " , np.array([self.y[t]]))
                # print("y " , self.h(next_x_hat_given_t))
                temp = np.array(self.y[t]) - self.h(next_x_hat_given_t)
                # print("temp " , temp)
                # print("k : ", K[t])
                
                next_xhat = (next_x_hat_given_t + K[t] @ (temp))

                if len(self.x_hat) - 1 < t + 1:
                    self.x_hat.append(next_xhat)
                else: self.x_hat[t + 1] = next_xhat
                
                #Covariance Update
                P.append((np.eye(6) - K[t] @ C_next) @ next_P_given_t)

                t += 1
        else:
            self.x_hat[t] = self.x[0]

        end_time = time.time()
        elapsed_time = end_time - start_time
        self.computation_time.append(elapsed_time)
        print(f"Average Execution time per step: {sum(self.computation_time)/len(self.computation_time)} seconds")
        print(f"Execution time: {elapsed_time} seconds")
        print("Accuracy: ", self.accuracy())
        return self.x_hat

    def g(self, x, u):
        


        # print("Ax : " ,self.approx_A(x,u)@x)
        # print("x : " , x)
        # print("Bu: " , self.approx_B(x,u)@u)
        # print("u : " , u)
        # print("E : " , self.approx_E(x,u))



        return self.approx_A(x,u) @ x + self.approx_B(x,u) @ u + self.approx_E(x,u)

    def h(self, x): # There was a y_obs I deleted, IDK why there is one
        # print("x is ", x)
        # print("x[0] is ", x[0])
        # print( "h func: ", np.sqrt((self.landmark[0] - x[0])**2 + self.landmark[1]**2 + (self.landmark[1] - x[1])**2))
        
        
        return np.array([np.sqrt((self.landmark[0] - x[0])**2 + self.landmark[1]**2 + (self.landmark[1] - x[1])**2), 
                                       x[2]])
    
    def h_prime(self, x):

        d_prime_dx = lambda x: 0.5 * ((self.landmark[0] - x[0])**2 + (self.landmark[2] - x[1])**2)**(-.5) * -2 * (self.landmark[0] - x[0])
        d_prime_dz = lambda x: 0.5 * ((self.landmark[0] - x[0])**2 + (self.landmark[2] - x[1])**2)**(-.5) * -2 * (self.landmark[2] - x[1])
        
        h = np.array([[d_prime_dx(x),  d_prime_dz(x), 0, 0, 0, 0],
                      [0               ,0           , 1, 0, 0, 0]])
        
        return h

    def approx_A(self, x, u): # Assuming no timestamp

        f_prime = np.array([[0, 0, 0,                       1, 0, 0], 
                            [0, 0, 0,                       0, 1, 0],
                            [0, 0, 0,                       0, 0, 1],
                            [0,0, -np.cos(x[2])/self.m*u[0],0, 0, 0],
                            [0,0,-np.sin(x[2])/self.m*u[0] , 0, 0, 0],
                            [0, 0, 0,                       0, 0, 0]])
        return np.eye(6) + f_prime * self.dt
                            
    def approx_B(self, x, u): # Assuming no timestamp
        f_prime = np.array([[0, 0,], 
                            [0, 0],
                            [0, 0],
                            [-np.sin(x[2])/self.m,0],
                            [np.cos(x[2])/self.m , 0],
                            [0, 1/self.J]])
        return f_prime * self.dt
    
    def approx_E(self, x, u): # Assuming no timestamp
        f_first = lambda x: np.array([[0, 0],
                                        [0, 0],
                                        [0, 0],
                                        [-np.sin(x[2])/self.m, 0],
                                        [np.cos(x[2])/self.m, 0],
                                        [0, 1/self.J]])
        
        f_second = lambda u: np.array([[u[0]],
                                        [u[1]]])

        f = lambda x, u: np.array([[x[3]],
                                [x[4]],
                                [x[5]],
                                [0],
                                [-self.gr],
                                [0]]) \
                                + np.dot(f_first(x), f_second(u))

        g = lambda x, u: np.array([[x[0]],
                                       [x[1]],
                                       [x[2]],
                                       [x[3]],
                                       [x[4]],
                                       [x[5]]]) \
                                       + f(x, u) * self.dt
        

        # print(" g : ", g(x,u).flatten())
        # print(" A in E: ", self.approx_A(x,u)@x)
        # print(" B in E: ", self.approx_B(x,u)@u)


        return g(x,u).flatten() - self.approx_A(x,u)@x - self.approx_B(x,u)@u


    
    def approx_C(self, x):
        # h_prime
        raise NotImplementedError
