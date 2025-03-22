import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np
plt.rcParams['font.family'] = ['FreeSans', 'Helvetica', 'Arial']
plt.rcParams['font.size'] = 14


class Estimator:
    """A base class to represent an estimator.

    This module contains the basic elements of an estimator, on which the
    subsequent DeadReckoning, Kalman Filter, and Extended Kalman Filter classes
    will be based on. A plotting function is provided to visualize the
    estimation results in real time.

    Attributes:
    ----------
        d : float
            Half of the track width (m) of TurtleBot3 Burger.
        r : float
            Wheel radius (m) of the TurtleBot3 Burger.
        u : list
            A list of system inputs, where, for the ith data point u[i],
            u[i][0] is timestamp (s),
            u[i][1] is left wheel rotational speed (rad/s), and
            u[i][2] is right wheel rotational speed (rad/s).
        x : list
            A list of system states, where, for the ith data point x[i],
            x[i][0] is timestamp (s),
            x[i][1] is bearing (rad),
            x[i][2] is translational position in x (m),
            x[i][3] is translational position in y (m),
            x[i][4] is left wheel rotational position (rad), and
            x[i][5] is right wheel rotational position (rad).
        y : list
            A list of system outputs, where, for the ith data point y[i],
            y[i][0] is timestamp (s),
            y[i][1] is translational position in x (m) when freeze_bearing:=true,
            y[i][1] is distance to the landmark (m) when freeze_bearing:=false,
            y[i][2] is translational position in y (m) when freeze_bearing:=true, and
            y[i][2] is relative bearing (rad) w.r.t. the landmark when
            freeze_bearing:=false.
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
        sub_u : rospy.Subscriber
            ROS subscriber for system inputs.
        sub_x : rospy.Subscriber
            ROS subscriber for system states.
        sub_y : rospy.Subscriber
            ROS subscriber for system outputs.
        tmr_update : rospy.Timer
            ROS Timer for periodically invoking the estimator's update method.

    Notes
    ----------
        The frozen bearing is pi/4 and the landmark is positioned at (0.5, 0.5).
    """
    # noinspection PyTypeChecker
    def __init__(self):
        self.d = 0.08
        self.r = 0.033
        self.u = []
        self.x = []
        self.y = []
        self.x_hat = []  # Your estimates go here!
        self.dt = 0.1
        self.fig, self.axd = plt.subplot_mosaic(
            [['xy', 'phi'],
             ['xy', 'x'],
             ['xy', 'y'],
             ['xy', 'thl'],
             ['xy', 'thr']], figsize=(20.0, 10.0))
        self.ln_xy, = self.axd['xy'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_xy_hat, = self.axd['xy'].plot([], 'o-c', label='Estimated')
        self.ln_phi, = self.axd['phi'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_phi_hat, = self.axd['phi'].plot([], 'o-c', label='Estimated')
        self.ln_x, = self.axd['x'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_x_hat, = self.axd['x'].plot([], 'o-c', label='Estimated')
        self.ln_y, = self.axd['y'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_y_hat, = self.axd['y'].plot([], 'o-c', label='Estimated')
        self.ln_thl, = self.axd['thl'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_thl_hat, = self.axd['thl'].plot([], 'o-c', label='Estimated')
        self.ln_thr, = self.axd['thr'].plot([], 'o-g', linewidth=2, label='True')
        self.ln_thr_hat, = self.axd['thr'].plot([], 'o-c', label='Estimated')
        self.canvas_title = 'N/A'
        self.sub_u = rospy.Subscriber('u', Float32MultiArray, self.callback_u)
        self.sub_x = rospy.Subscriber('x', Float32MultiArray, self.callback_x)
        self.sub_y = rospy.Subscriber('y', Float32MultiArray, self.callback_y)
        self.tmr_update = rospy.Timer(rospy.Duration(self.dt), self.update)

    def callback_u(self, msg):
        self.u.append(msg.data)

    def callback_x(self, msg):
        self.x.append(msg.data)
        if len(self.x_hat) == 0:
            self.x_hat.append(msg.data)

    def callback_y(self, msg):
        self.y.append(msg.data)

    def update(self, _):
        raise NotImplementedError

    def plot_init(self):
        self.axd['xy'].set_title(self.canvas_title)
        self.axd['xy'].set_xlabel('x (m)')
        self.axd['xy'].set_ylabel('y (m)')
        self.axd['xy'].set_aspect('equal', adjustable='box')
        self.axd['xy'].legend()
        self.axd['phi'].set_ylabel('phi (rad)')
        self.axd['phi'].legend()
        self.axd['x'].set_ylabel('x (m)')
        self.axd['x'].legend()
        self.axd['y'].set_ylabel('y (m)')
        self.axd['y'].legend()
        self.axd['thl'].set_ylabel('theta L (rad)')
        self.axd['thl'].legend()
        self.axd['thr'].set_ylabel('theta R (rad)')
        self.axd['thr'].set_xlabel('Time (s)')
        self.axd['thr'].legend()
        plt.tight_layout()

    def plot_update(self, _):
        self.plot_xyline(self.ln_xy, self.x)
        self.plot_xyline(self.ln_xy_hat, self.x_hat)
        self.plot_philine(self.ln_phi, self.x)
        self.plot_philine(self.ln_phi_hat, self.x_hat)
        self.plot_xline(self.ln_x, self.x)
        self.plot_xline(self.ln_x_hat, self.x_hat)
        self.plot_yline(self.ln_y, self.x)
        self.plot_yline(self.ln_y_hat, self.x_hat)
        self.plot_thlline(self.ln_thl, self.x)
        self.plot_thlline(self.ln_thl_hat, self.x_hat)
        self.plot_thrline(self.ln_thr, self.x)
        self.plot_thrline(self.ln_thr_hat, self.x_hat)

    def plot_xyline(self, ln, data):
        if len(data):
            x = [d[2] for d in data]
            y = [d[3] for d in data]
            ln.set_data(x, y)
            self.resize_lim(self.axd['xy'], x, y)

    def plot_philine(self, ln, data):
        if len(data):
            t = [d[0] for d in data]
            phi = [d[1] for d in data]
            ln.set_data(t, phi)
            self.resize_lim(self.axd['phi'], t, phi)

    def plot_xline(self, ln, data):
        if len(data):
            t = [d[0] for d in data]
            x = [d[2] for d in data]
            ln.set_data(t, x)
            self.resize_lim(self.axd['x'], t, x)

    def plot_yline(self, ln, data):
        if len(data):
            t = [d[0] for d in data]
            y = [d[3] for d in data]
            ln.set_data(t, y)
            self.resize_lim(self.axd['y'], t, y)

    def plot_thlline(self, ln, data):
        if len(data):
            t = [d[0] for d in data]
            thl = [d[4] for d in data]
            ln.set_data(t, thl)
            self.resize_lim(self.axd['thl'], t, thl)

    def plot_thrline(self, ln, data):
        if len(data):
            t = [d[0] for d in data]
            thr = [d[5] for d in data]
            ln.set_data(t, thr)
            self.resize_lim(self.axd['thr'], t, thr)

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
        $ roslaunch proj3_pkg unicycle_bringup.launch \
            estimator_type:=oracle_observer \
            noise_injection:=true \
            freeze_bearing:=false
    """
    def __init__(self):
        super().__init__()
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
        $ roslaunch proj3_pkg unicycle_bringup.launch \
            estimator_type:=dead_reckoning \
            noise_injection:=true \
            freeze_bearing:=false
    For debugging, you can simulate a noise-free unicycle model by setting
    noise_injection:=false.
    """
    def __init__(self):
        super().__init__()
        self.canvas_title = 'Dead Reckoning'

    # u : list
        # A list of system inputs, where, for the ith data point u[i],
        # u[i][0] is timestamp (s),
        # u[i][1] is left wheel rotational speed (rad/s), and
        # u[i][2] is right wheel rotational speed (rad/s).

    # x : list
    #     A list of system states, where, for the ith data point x[i],
    #     x[i][0] is timestamp (s),
    #     x[i][1] is bearing (rad),
    #     x[i][2] is translational position in x (m),
    #     x[i][3] is translational position in y (m),
    #     x[i][4] is left wheel rotational position (rad), and
    #     x[i][5] is right wheel rotational position (rad).

    # g(x, u) = x + f(x, u) * delta t

    # x  = [phi, x, y, theta_L, theta_R].T
    # u = [u_L, u_R]

    def update(self, _):
        if len(self.x_hat) > 0 and self.x_hat[-1][0] < self.x[-1][0]:
            # TODO: Your implementation goes here!
            # You may ONLY use self.u and self.x[0] for estimation
            f_first = lambda x: np.array([[-self.r/(2*self.d), self.r/(2*self.d)], 
                                          [(self.r/2)*np.cos(x[1]), (self.r/2)*np.cos(x[1])],
                                          [(self.r/2)*np.sin(x[1]), (self.r/2)*np.sin(x[1])],
                                          [1, 0],
                                          [0, 1]])
            
            f_second = lambda u: np.array([[u[1]],
                                           [u[2]]])
            
            f = lambda x, u: np.dot(f_first(x), f_second(u))
            g = lambda x, u: np.array([[x[1]],
                                       [x[2]],
                                       [x[3]],
                                       [x[4]],
                                       [x[5]]]) \
                                       + f(x, u) * self.dt

            T = len(self.u)
            t = 0
            timestamp = self.x[0][0]
            self.x_hat.append(self.x[0])

            while t < T - 1:
                next_xhat = np.array([timestamp]).append(g(self.x_hat[t], self.u[t]).reshape(6))
                if len(self.x_hat) - 1 < t + 1:
                    self.x_hat.append(next_xhat)
                else: self.x_hat[t + 1] = next_xhat
                t = t + 1
        else:
            self.x_hat[0] = self.x[0]


class KalmanFilter(Estimator):
    """Kalman filter estimator.
/
    Your task is to implement the update method of this class using the u
    attribute, y attribute, and x0. You will need to build a model of the
    linear unicycle model at the default bearing of pi4. After building the
    model, use the provided inputs and outputs to estimate system state over
    time via the recursive Kalman filter update rule.

    Attributes:
    ----------
        phid : float
            Default bearing of the turtlebot fixed at pi / 4.

    Example
    ----------
    To run the Kalman filter:
        $ roslaunch proj3_pkg unicycle_bringup.launch \
            estimator_type:=kalman_filter \
            noise_injection:=true \
            freeze_bearing:=true
    """
    def __init__(self):
        super().__init__()
        self.canvas_title = 'Kalman Filter'
        self.phid = np.pi/4
        # TODO: Your implementation goes here!
        # State transition matrix: 4x4
        self.A = np.eye(4)

        # Measurement matrix: 2x4 (measuring position only)
        self.C = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # Covariance of process noise: 4x4
        self.Q = 1 * np.eye(4)

        # Covariance of measurement noise: 2x2
        self.R = 0.01 * np.eye(2)

        # Initial covariance: 4x4
        self.P_0 = 1000 * np.eye(4)
        # You may define the A, C, Q, R, and P matrices below.

    # noinspection DuplicatedCode
    # noinspection PyPep8Naming
    def update(self, _):
        if len(self.x_hat) > 0 and self.x_hat[-1][0] < self.x[-1][0]:
            # TODO: Your implementation goes here!
            # You may use self.u, self.y, and self.x[0] for estimation

            t = 0
            self.x_hat[t] = self.x[0]
            P = [self.P_0.copy()]
            K = []
            timestamp = self.x[0][0]


            # control_t = np.array([50, 50])
            # print("x_hat: ",  self.x_hat, "\n")
            T = len(self.u)
            # print("u: ",  self.u, "\n")
            while t < T:
                self.B = self.dt * np.array([
                    [self.r/2*np.cos(self.phid) , self.r/2*np.cos(self.phid)],
                    [self.r/2*np.sin(self.phid) , self.r/2*np.sin(self.phid)],
                    [1                          , 0],
                    [0                          , 1],
                ])
                # State Extrapolation
                # print("x_hat: ",  self.x_hat, "\n")
                next_x_hat_given_t = self.A @ self.x_hat[t][2:] + self.B @ self.u[t][1:]
                
                #covariance extrapolation
                # print("This is P: ", P)
                next_P_given_t = self.A @ P[t] @ self.A.T + self.Q

                # Kalman Gain
                k_gain = np.linalg.inv(self.C @ next_P_given_t @ self.C.T + self.R)
                K.append(next_P_given_t @ self.C.T @ k_gain)
                # print("k : ", K)
                # print("y : ", self.y)

                # State Update
                x_upd = next_x_hat_given_t + K[t] @ (self.y[t][2:] - self.C @ next_x_hat_given_t)


                self.x_hat.append(np.concatenate(([self.u[t][0], self.x_hat[t][1]], x_upd), axis=None))
                print("pos : ",  self.x_hat[t])
                
                #Covariance Update
                P.append((np.eye(4) - K[t] @ self.C) @ next_P_given_t)

                t += 1
            
        return self.x_hat


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

    Example
    ----------
    To run the extended Kalman filter:
        $ roslaunch proj3_pkg unicycle_bringup.launch \
            estimator_type:=extended_kalman_filter \
            noise_injection:=true \
            freeze_bearing:=false
    """
    def __init__(self):
        super().__init__()
        self.canvas_title = 'Extended Kalman Filter'
        self.landmark = (0.5, 0.5)

        # Measurement matrix: 2x4 (measuring position only)
        self.C = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # Covariance of process noise: 4x4
        self.Q = 0.01 * np.eye(5)

        # Covariance of measurement noise: 2x2
        self.R = 0.1 * np.eye(2)

        # Initial covariance: 4x4
        self.P_0 = 100 * np.eye(5)
        # You may define the Q, R, and P matrices below.

    # noinspection DuplicatedCode
    def update(self, _):
        if len(self.x_hat) > 0 and self.x_hat[-1][0] < self.x[-1][0]:
            t = 0
            self.x_hat[t] = self.x[0]
            P = [self.P_0.copy()]
            K = []

            f_first = lambda x: np.array([[-self.r/(2*self.d), self.r/(2*self.d)], 
                            [(self.r/2)*np.cos(x[1]), (self.r/2)*np.cos(x[1])],
                            [(self.r/2)*np.sin(x[1]), (self.r/2)*np.sin(x[1])],
                            [1, 0],
                            [0, 1]])
        
            f_prime = lambda x, u: np.array([[0, 0, 0, 0, 0], 
                            [-(self.r/2)*np.sin(x[1])*u[1]-(self.r/2)*np.sin(x[1])*u[2], 0, 0, 0, 0],
                            [(self.r/2)*np.cos(x[1])*u[1] + (self.r/2)*np.cos(x[1])*u[2], 0, 0, 0, 0],
                            [0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0]])

            f_second = lambda u: np.array([[u[1]],
                                        [u[2]]])
            
            f = lambda x, u: np.dot(f_first(x), f_second(u))
            g = lambda x, u: np.array([[x[1]],
                                    [x[2]],
                                    [x[3]],
                                    [x[4]],
                                    [x[5]]]) \
                                    + f(x, u) * self.dt

            g_prime = lambda x, u: np.eye(5) + f_prime(x, u) * self.dt

            d_prime_dx = lambda x: 0.5 * ((self.landmark[0] - x[1])**2 + (self.landmark[1] - x[2])**2)**(-.5) * -2 * (self.landmark[0] - x[1])
            d_prime_dy = lambda x: 0.5 * ((self.landmark[0] - x[1])**2 + (self.landmark[1] - x[2])**2)**(-.5) * -2 * (self.landmark[0] - x[2])
        

            h_prime = lambda x: np.array([[0, d_prime_dx(x)[0],  d_prime_dy(x)[0], 0, 0],
                                    [1, 0, 0, 0 ,0 ]])

            h = lambda x: np.array([np.sqrt((self.landmark[0] - x[1])**2 + (self.landmark[1] - x[2])**2), 
                                       x[0]])

            # control_t = np.array([50, 50])
            # print("x_hat: ",  self.x_hat, "\n")
            T = len(self.u)
            # print("u: ",  self.u, "\n")
            while t < T:
                # State Extrapolation
                # print("x_hat: ",  self.x_hat, "\n")

 
                next_x_hat_given_t = g(self.x_hat[t], self.u[t])

                A_next = g_prime(self.x_hat[t], self.u[t])
                # print("A_next ", A_next)

                next_P_given_t = A_next @ (P[t]) @ A_next.T + self.Q
                # print("P  ", next_P_given_t)
                
                C_next = h_prime(next_x_hat_given_t) # h_prime[x,y] values

                # print("c  ", C_next)

                k_gain = np.linalg.inv(C_next @ next_P_given_t @ C_next.T + self.R)
                K.append(next_P_given_t @ C_next.T @ k_gain)
                # print("y : ", self.y)

                # State Update
                temp = np.array([self.y[t][1:]]).T - h(next_x_hat_given_t)

                # print("y " , h(next_x_hat_given_t))
                # print("temp " , temp)
                # print("k : ", K[t])
                
                x_upd = next_x_hat_given_t + K[t] @ (temp)

                # print("pos : ",  x_upd)

                self.x_hat.append(np.concatenate((np.array([self.u[t][0]]), x_upd.flatten())))
                print("pos : ",  self.x_hat[t])
                
                #Covariance Update
                P.append((np.eye(5) - K[t] @ C_next) @ next_P_given_t)

                t += 1
