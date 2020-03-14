'''
Author: Elvis Rodas.

'''

import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from filterpy.kalman import KalmanFilter
from tf.transformations import euler_from_quaternion


class Kalman:
    '''
        The Kalman Filter
        To build the kalman filter it is important to provide the measurement values and the variance of respective
        state variable.
        Data in standard units. [meters,secs, etc]

        Equation used is:
                            x = F * x  + B * u + w      ----- (1)
                            z = H * x + v               ----- (2)

        TODO: Explain the kalman filter algorithm.

        Args:
            odom (Odometry msg) : The odometry data containing the Pose (position, orientation and measurement covariance
                                    of respective variable) and Twist (Linear and angluar velocity with covariance).
                                    More info: http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html

            imu (Imu msg)       : The Imu data containing the angular and linear acceleration with the measurement covariance.
                                    More info: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html

            process_var         : Process variance. A dictionary holding the diagonal elements of the 8x8 process covariance matrix.
                                  format: { process_cov_xpos: ...,
                                            process_cov_ypos: ...,
                                            process_cov_xvel: ...,
                                            process_cov_yvel: ...,
                                            process_cov_xacc: ...,
                                            process_cov_yacc: ...,
                                            process_cov_yaw : ...,
                                            process_cov_yawvel: .. }

            only_uwb (Boolean)  : jUse only location data. This is important to construct dimention of state space.

            OBS: The measurement covariance matrix R will be built from the covariance stored in 'odom' and 'imu'.
                  It is expected that the ROS msgs have those values.

        Attributes:
            kf: The kalman Filter object created from filterpy.kalman module.

        Public Methods:
            kalman_predic:  perform predicition.
            kalman_update: perform update.

        Private Methods:
            setX    : Sets the state vector from eq. (1)-(2)
            setF    : Sets the process matrix F matrix from eq. (1)
            setB    : Sets the control matrix B from eq. (1)
            setH    : Sets the measurement matrix H from eq. (2)
            setQ    : Sets the process covariance matrix Q needed for the kalman filter.
            setR    : Sets the measruements covariance matrix R needed for the kalman filter.

    '''

    def __init__(self, odom, imu, dt, only_uwb=False):
        # TODO: Explanation 'c'
        c = 0.9  # Speed divided by length.

        self.only_uwb = only_uwb

        # Update dimentions. u - control vector. x - state space vars. z - measurement vector.
        if self.only_uwb:
            dim_u = 0   # No control.
            dim_x = 4   # [x, y, yaw, x_vel, y_vel]
            dim_z = 2   # [x, y, -  ,  -   ,   -  ]
        else:
            dim_u = 1   # yaw control.
            dim_x = 8   # [x, y, yaw, yaw_vel, x_vel, y_vel, x_accel, y_accel]
            dim_z = 5   # [x, y, yaw,   -    ,  -   , -    , x_accel, y_accel]

        # initialize Kalman filter
        self.kf = KalmanFilter(dim_x=dim_x, dim_z=dim_z, dim_u=dim_u)

        # init state vector and matrices.
        self.kf.X = self.__setX(odom.pose.pose.position)
        self.kf.F = self.__setF(dt)
        self.kf.B = self.__setB(c)
        self.kf.H = self.__setH()

        # Filter variables
        self.kf.P *= 10
        # measurement noise covariance
        self.kf.R = self.__setR(odom.pose.covariance, odom.twist.covariance,
                                imu.angular_velocity_covariance, imu.linear_acceleration_covariance)
        # Diagonal elements of process covariance matrix.
        # TODO: let user input the process covariance matrix Q.
        defaultQ = np.array([.0, .0, .0, .08, .1, .1,  .08, .08])
        self.kf.Q = self.__setQ(dt, defaultQ)  # process noise

    def __setX(self, position):
        '''
            setX. Private method initiates the state space.
            if only uwb: [x, y]
                else:   [x, y, yaw, yaw_vel, x_vel, y_vel, x_accel, y_accel]
            Args:
                position: A geometry_msgs/Point msg containing the x,y,z position.
            Returns:
                X: a nx1 matrix containing the state space [x,y,theta, theta_dot, xdot, ydot, xddot, yddot]^T

            TODO: Initiatate also data from IMU.
        '''

        if self.only_uwb:
            X = np.zeros([8, 1])
            X[0, 0] = position.x
            X[1, 0] = position.y
        else:
            # initialize with first position x and y, 0 in v and a
            X = np.array([[position.x], [position.y], [0.0], [0.0]])
        return X

    def __setF(self, dt):
        '''
            Args:
                dt: The time step.
            Returns:
                F: The process matrix.
        '''
        if self.only_uwb:  # dim_x = 6, dim_z = 4
            F = np.array([[1., 0., 0., 0,  dt, 0., 0.5 * dt * dt, 0.],
                          [0., 1., 0., 0., 0., dt, 0., 0.5 * dt * dt],
                          [0., 0., 1., dt, 0., 0., 0., 0.],
                          [0., 0., 0., 1., 0., 0., 0., 0.],
                          [0., 0., 0., 0., 1., 0., dt, 0.],
                          [0., 0., 0., 0., 0., 1., 0., dt],
                          [0., 0., 0., 0., 0., 0., 1., 0.],
                          [0., 0., 0., 0., 0., 0., 0., 1.]]
                         )
        else:
            F = np.array([[1., 0., dt, 0],
                          [0., 1., 0, dt],
                          [0., 0., 1., 0.],
                          [0., 0., 0., 1.]])

        return F

    def __setB(self, c):
        '''
            Args:
                c : The speed divided by length v/L according to Ackerman equation.
            Returns:
                B : The B matrix.
        '''
        # TODO: Make this general.
        c = 0.8  # v/L.

        if self.only_uwb:
            B = np.zeros([8, 1])
            B[3, 0] = c
        else:
            B = 0
        return B

    def __setH(self):
        if self.only_uwb:
            H = np.array([[1., 0., 0., 0., 0., 0., 0., 0.],
                          [0., 1., 0., 0., 0., 0., 0., 0.],
                          [0., 0., 1., 0., 0., 0., 0., 0.],
                          [0., 0., 0., 0., 0., 0., 1., 0.],
                          [0., 0., 0., 0., 0., 0., 0., 1.]])
        else:
            H = np.array([[1., 0., 0., 0.],
                          [0., 1., 0., 0.]])
        return H

    def __setQ(self, processQ):
        '''
        setQ: Private method that computes the 8x8 process covariance matrix Q.

            Args:
                processQ: The process covariance matrix. Expecting row vector.
                          TODO: If arg is a 1xn vector, assume diagonal elements are given, else a square matrix must be given.
            Returns:
                Q: The process noise covariance matrix.

        '''

        if self.only_uwb:

            Q = np.diag(processQ)
        else:
            var_x_dot = processQ[0]
            var_y_dot = processQ[1]
            Q = np.diag(np.array([var_x_dot, var_y_dot]))

        return Q

    def __setR(self, pos_var, vel_var, angular_accel_var, linear_accel_var):
        '''
        SetR: Private method. Computes measurement noise matrix R.
            Args:
                pos_var             : 6x6 diagonal matrix. Position (x,y,z) variance.
                                      OBS: This matrix include also the variance of the orientation (raw, pitch, yaw),
                                      because of the structure of the 'Pose' ROS msg. Only the variance of x,y will be used here.
                                      (See http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseWithCovariance.html)

                vel_var             : 6x6 diagonal matrix. Velocity (vx,vy,vz) variance.
                angular_accel_var   : 3x3 diagonal matrix. Angular acceleration (a_roll, a_pitch, a_yaw) variance.
                linear_accel_var    : 3x3 diagonal matrix. Linear acceleration (ax, ay, az).

            Returns:
                The measurement covariance matrix R.
        '''
        var_x = pos_var[0 + 6*0]
        var_y = pos_var[1 + 6*1]
        var_yaw = angular_accel_var[2 + 3*2]
        var_x_accel = linear_accel_var[0 + 3*0]
        var_y_accel = linear_accel_var[1 + 3*0]

        if self.only_uwb:

            R = np.diag(
                np.array([var_x, var_y, var_yaw, var_x_accel, var_y_accel]))
        else:
            R = np.diag(np.array([var_x, var_y]))

        return R

    def kalman_predic(self):
        '''
        predic. Public method. Performs the first step of KF.

        '''
        self.kf.predic()

    def kalman_update(self, odom, imu):
        '''
        Update. Public method. Performs the second step of KF.

        Args:
            position: A Pose message (geometry_msgs/Pose.msg)
            imu     : An Imu message (sensor_msgs/Imu.msg)
        Returns:
            None

        '''
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y

        # Converting quaterions to euler angles.
        [roll, pitch, yaw] = euler_from_quaternion(imu.orientation)

        # TODO: if yaw < 0: yaw += 360
        yaw_vel = imu.angular_velocity.z

        xddot = imu.linear_acceleration.x
        yddot = imu.linear_acceleration.y

        self.kf.update(
            np.array([[x], [y], [yaw], [yaw_vel], [xddot], [yddot]]))

    def get_estimation(self):
        return self.x
